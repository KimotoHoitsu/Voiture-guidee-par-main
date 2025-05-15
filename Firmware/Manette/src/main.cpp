#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLE2902.h>

// ==========================
// Configuration & Defines
// ==========================

// ---- Motor Control Pins ----
#define MOTOR1_IN1 7    // Motor 1, input 1
#define MOTOR1_IN2 6    // Motor 1, input 2
#define MOTOR2_IN1 11   // Motor 2, input 1
#define MOTOR2_IN2 13   // Motor 2, input 2

// ---- Proximity Sensor Pins ----
#define ECHO1 1       // bottom left
#define TRIG1 14
#define ECHO2 2       // bottom right
#define TRIG2 21
#define ECHO3 9       // top left
#define TRIG3 10
#define ECHO4 3       // top right
#define TRIG4 8


// ---- Encoder Pins ----
#define ENCODER_LEFT_A 2    // Left encoder channel A
#define ENCODER_LEFT_B 3    // Left encoder channel B
#define ENCODER_RIGHT_A 4   // Right encoder channel A
#define ENCODER_RIGHT_B 5   // Right encoder channel B

// ---- PWM Configuration for Motors & Servo ----
#define PWM_CHANNEL_1 0   // Motor side 1 forward
#define PWM_CHANNEL_2 1   // Motor side 1 reverse
#define PWM_CHANNEL_3 2   // Motor side 2 (right) forward
#define PWM_CHANNEL_4 3   // Motor side 2 (right) reverse

#define PWM_FREQUENCY 5000   // Motor PWM frequency (Hz)
#define PWM_RESOLUTION 10    // Motor PWM resolution (10-bit)

#define PWM_CHANNEL_SERVO 5         // Servo PWM channel
#define PWM_FREQUENCY_SERVO 50      // Servo PWM frequency (Hz)
#define PWM_RESOLUTION_SERVO 10     // Servo PWM resolution
#define SERVO_PIN 12                // Servo control pin

// Servo PWM limits
#define MAX_PWM 100
#define MIN_PWM 50
#define NEUTRAL_PWM 77

// ---- CMPS12 (Compass) ----
#define CMPS12_ADDR 0x60   // I2C address

// ---- BLE Settings ----
#define SERVICE_UUID "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-210987654321"

// ---- Motor Side Identifiers ----
#define SIDE_LEFT 1
#define SIDE_RIGHT 2

// ---- Crash Avoidance ----
#define COLLISION_DISTANCE 15  //  cm threshold for collision detection

// ==========================
// Global Variables & BLE Objects
// ==========================
BLEClient *pClient = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
bool deviceConnected = false;

float xValue = 0.0;  // Global variable for BLE x-axis control
float yValue = 0.0;  // Global variable for BLE y-axis control
float zValue = 0.0;  // Global variable for BLE z-axis control

// ----- Encoder & Drive Variables -----
// For your 550 RPM / 1:19 motor using single-edge counting:
// Raw encoder PPR = 11; Effective counts per wheel revolution = 11 x 19 = 209.
const int EFFECTIVE_ENCODER_COUNTS = 209;  // Use this instead of ENCODER_PPR
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

// For RPM calculation
unsigned long prevTime = 0;
long prevEncoderCount = 0;
float rightRPM = 0.0;

const uint8_t echoPins[4] = {  ECHO1, ECHO2, ECHO3, ECHO4 };   // sensor 1-4 ECHO
const uint8_t trigPins[4] = {  TRIG1, TRIG2, TRIG3, TRIG4 };   // sensor 1-4 TRIG

// ==========================
// ISR Functions
// ==========================
void IRAM_ATTR encoderRight_ISR() {
  encoderRightCount++;
}

void IRAM_ATTR encoderLeft_ISR() {
  encoderLeftCount++;
}

// ==========================
// Utility Functions
// ==========================

// Simplify critical section: Read volatile variable safely.
long getEncoderRightCount() {
  noInterrupts();
  long count = encoderRightCount;
  interrupts();
  return count;
}

// Motor movement using PWM
void motor_move(int side, int pwm) {
  int channel1 = (side == SIDE_LEFT) ? PWM_CHANNEL_1 : PWM_CHANNEL_4;
  int channel2 = (side == SIDE_LEFT) ? PWM_CHANNEL_2 : PWM_CHANNEL_3;
  
  if (pwm >= 0) {
    ledcWrite(channel1, pwm);
    ledcWrite(channel2, 0);
  } else {
    ledcWrite(channel1, 0);
    ledcWrite(channel2, abs(pwm));
  }
}

// Servo movement (adjust PWM based on input value)
void moveServo(int pwmValue) {
  int targetPWM = map(constrain(pwmValue, -10, 10), -10, 10, MIN_PWM, MAX_PWM);
  static int currentPWM = NEUTRAL_PWM;
  if (currentPWM != targetPWM) {
    currentPWM = targetPWM;
    ledcWrite(PWM_CHANNEL_SERVO, currentPWM);
  }
}

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return (duration == 0) ? -1 : duration / 58.2;
}

float measureDistanceSensor(uint8_t sensor)   // sensor = 1 … 4
{
    if (sensor < 1 || sensor > 4) {           // quick bounds check
        return -1;                            // invalid sensor ID
    }

    // subtract 1 because arrays are 0-indexed
    uint8_t trigPin = trigPins[sensor - 1];
    uint8_t echoPin = echoPins[sensor - 1];

    return measureDistance(trigPin, echoPin); // call your original helper
}

// ==========================
// Crash Avoidance Function
// ==========================
void avoidCrash(float &yValue) {
  // Check distances for all sensors
  float distances[4];
  for (int i = 0; i < 4; i++) {
    distances[i] = measureDistanceSensor(i + 1);
  }

   // Debugging: Print sensor readings to the Serial Monitor
   //Serial.print("Sensor Distances (cm): ");
   //Serial.print("Bottom Left: "); Serial.print(distances[0]); Serial.print(" | ");
   //Serial.print("Bottom Right: "); Serial.print(distances[1]); Serial.print(" | ");
   //Serial.print("Top Left: "); Serial.print(distances[2]); Serial.print(" | ");
   //Serial.print("Top Right: "); Serial.println(distances[3]);

  // Override yValue based on proximity sensor readings
  if (distances[2] > 0 && distances[2] < COLLISION_DISTANCE) {  // Top Left
    yValue += 5;
  } else if (distances[3] > 0 && distances[3] < COLLISION_DISTANCE) {  // Top Right
    yValue -= 5;
  } else if (distances[0] > 0 && distances[0] < COLLISION_DISTANCE) {  // Bottom Left
    yValue += 5;
  } else if (distances[1] > 0 && distances[1] < COLLISION_DISTANCE) {  // Bottom Right
    yValue -= 5;
  }
}


// ==========================
// BLE Callback Class
// ==========================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) { deviceConnected = true; }
  void onDisconnect(BLEClient *pclient) { deviceConnected = false; }
};

// ==========================
// BLE Notification Callback
// ==========================
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData, size_t length, bool isNotify) {
  String data = String((char*)pData);
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);

  float xValue = data.substring(0, firstComma).toFloat();
  float yValue = data.substring(firstComma + 1, secondComma).toFloat();
  float zValue = data.substring(secondComma + 1, thirdComma).toFloat();

  // Apply crash avoidance logic
  avoidCrash(yValue);

  // Motor control based on xValue
  int pwmValue = map(xValue, -10, 10, 1023, -1023);
  motor_move(SIDE_LEFT, pwmValue);
  motor_move(SIDE_RIGHT, pwmValue); 

  moveServo(yValue);

}

// ==========================
// Setup Function
// ==========================
void setup() {
  Serial.begin(115200);
  Wire.begin(17, 18);

  // Set up encoder pins
  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);
  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);

  // Set up proximity sensor pins
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);
  pinMode(TRIG4, OUTPUT); pinMode(ECHO4, INPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeft_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRight_ISR, RISING);

  // Initialize BLE
  BLEDevice::init("MotorController");
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  BLEAddress serverAddress("f4:12:fa:ae:1a:89");
  pClient->connect(serverAddress);
  pClient->setMTU(512);

  BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService) {
    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic)
      pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  // Set up PWM channels for Servo & Motors
  ledcSetup(PWM_CHANNEL_SERVO, PWM_FREQUENCY_SERVO, PWM_RESOLUTION_SERVO);
  ledcAttachPin(SERVO_PIN, PWM_CHANNEL_SERVO);
  
  ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_IN1, PWM_CHANNEL_1);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_IN2, PWM_CHANNEL_2);
  ledcSetup(PWM_CHANNEL_3, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_IN1, PWM_CHANNEL_3);
  ledcSetup(PWM_CHANNEL_4, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_IN2, PWM_CHANNEL_4);

  prevTime = millis();
  prevEncoderCount = encoderRightCount;  // For RPM calculation
}

// ==========================
// Main Loop
// ==========================
void loop() {
  
 // float d1 = measureDistanceSensor(1);
 // float d2 = measureDistanceSensor(2);
 // float d3 = measureDistanceSensor(3);
 // float d4 = measureDistanceSensor(4);
 //  Serial.printf("S1: %5.1f cm   S2: %5.1f cm   S3: %5.1f cm   S4: %5.1f cm\n",
  //              d1, d2, d3, d4);
  //delay(300); 
  unsigned long currentTime = millis();

  // Update RPM measurement every second
  if (currentTime - prevTime >= 1000) {
    long currentCount = getEncoderRightCount();
    long pulseDelta = currentCount - prevEncoderCount;
    prevEncoderCount = currentCount;
    
    // Calculate RPM: revolutions per minute = (pulseDelta / effective_counts) * 60
    rightRPM = ((float)pulseDelta / EFFECTIVE_ENCODER_COUNTS) * 60.0;
    float speedPercentage = (rightRPM / 520.0) * 100.0;
    
    if (deviceConnected && pRemoteCharacteristic) {
      String dataString = String(speedPercentage, 1) + "," + String(rightRPM, 2);
      pRemoteCharacteristic->writeValue(dataString.c_str());
    }
    prevTime = currentTime;
  }

  // Attempt BLE reconnection if disconnected
  if (!deviceConnected) {
    pClient->connect(BLEAddress("f4:12:fa:ae:1a:89"));
  }
}
