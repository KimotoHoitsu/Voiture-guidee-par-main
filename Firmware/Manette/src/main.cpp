#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <WiFi.h> // Include the WiFi library

// Create an instance of the ADXL343 class
Adafruit_ADXL343 adxl = Adafruit_ADXL343(12345);

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128
#define OLED_RESET -1
#define I2C_ADDRESS 0X3C

Adafruit_SSD1327 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SERVICE_UUID "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-210987654321"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Motor speed variable
float motorSpeed = 0.0;  // Store motor speed (%) from client

//timer for updating display
unsigned long previousDisplayMillis = 0;
const long displayInterval = 1000;  // 1000 ms = 1 second

// Callback to handle incoming notifications from the motor client
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData, size_t length, bool isNotify)
{
    String data = String((char *)pData);
    motorSpeed = data.toFloat();  // Update motor speed with the received value
    Serial.println("Motor Speed Received: " + String(motorSpeed) + "%");
}

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.println("Device connected.");
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("Device disconnected.");
        pServer->startAdvertising();
    }
};

void setup()
{
    Serial.begin(115200);
    Wire.begin(8, 9);

    // Initialize ADXL343 sensor
    if (!adxl.begin())
    {
        Serial.println("Failed to initialize ADXL343 sensor.");
        while (1)
            ;
    }
    adxl.setRange(ADXL343_RANGE_2_G); // Set the range to ±2g

    // Initialize OLED Display
    if (!display.begin(I2C_ADDRESS))
    {
        Serial.println("SSD1327 OLED init failed");
        while (1)
            ;
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(10, 10);
    display.print("ADXL343 Ready!");
    display.display();
    delay(1000);

    // Initialize BLE
    BLEDevice::init("RCgogo");
    BLEServer *pServer = BLEDevice::createServer(); // Create a BLE server
    pServer->setCallbacks(new MyServerCallbacks()); // Set server callbacks

    BLEService *pService = pServer->createService(SERVICE_UUID); // Create a BLE service

    // Create a BLE characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting for a client to connect...");

    // Send initial command to clear display
    // Clear OLED screen
    display.clearDisplay();

    // Set text properties
    display.setCursor(20, 20); // Adjust position
    display.setTextSize(2);    // Larger font size
    display.setTextColor(SSD1327_WHITE);

    // Update the display
    display.display();

    // Small delay to allow the message to be visible
    delay(50);
}

void loop()
{
  if (deviceConnected)
  {
      // Read accelerometer data
      sensors_event_t event;
      adxl.getEvent(&event);

      // Convert accelerometer values to string format
      String accelerometerData = String(event.acceleration.x, 2) + "," + String(event.acceleration.y, 2);

      // Send the ADXL data to the BLE client
      pCharacteristic->setValue(accelerometerData.c_str());
      pCharacteristic->notify();

      // Check if 1 second has passed
      unsigned long currentMillis = millis();
      if (currentMillis - previousDisplayMillis >= displayInterval)
      {
          previousDisplayMillis = currentMillis;

          // Clear OLED screen
          display.clearDisplay();

          // Display battery voltage
          display.setCursor(0, 10);
          display.setTextSize(1);
          display.setTextColor(SSD1327_WHITE);
          display.print("Battery: ");
          display.print(batteryPercentage);
          display.print("%");

          // Display accelerometer data
          display.setCursor(0, 30);
          display.print("X: ");
          display.print(event.acceleration.x, 2);

          display.setCursor(0, 45);
          display.print("Y: ");
          display.print(event.acceleration.y, 2);

          // Display motor speed
          display.setCursor(0, 60);
          display.print("Motor Speed: ");
          display.print(motorSpeed);

          display.display();  // Update OLED

          // Print to Serial
          Serial.println("---- Status Update ----");
          Serial.print("Battery: ");
          Serial.print(batteryPercentage);
          Serial.println("%");

          Serial.print("X: ");
          Serial.println(event.acceleration.x, 2);
          Serial.print("Y: ");
          Serial.println(event.acceleration.y, 2);
          Serial.print("Motor Speed: ");
          Serial.print(motorSpeed);
          Serial.println("%");
          Serial.println("-----------------------");
      }
  }
  else
  {
      unsigned long currentMillis = millis();
      if (currentMillis - previousDisplayMillis >= displayInterval)
      {
          previousDisplayMillis = currentMillis;
          display.clearDisplay();
          display.setCursor(0, 10);
          display.setTextSize(1);
          display.setTextColor(SSD1327_WHITE);
          display.print("Disconnected");
          display.display();
      }
  }
}
