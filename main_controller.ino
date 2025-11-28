#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// --- Pin Definitions ---
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

// L298N Channel A: Conveyor
#define CONVEYOR_ENA 6
#define CONVEYOR_IN1 25
#define CONVEYOR_IN2 24

// L298N Channel B: Vacuum Pump
#define VACUUM_ENB 5 
#define VACUUM_IN1 23 
#define VACUUM_IN2 22 

#define CAM_TX 19
#define CAM_TRIGGER_PIN 32
#define SD_CS 4
#define ULTRASONIC_TRIG_PIN 38
#define ULTRASONIC_ECHO_PIN 37
#define CURRENT_SENSOR_PIN A3 
#define FSR_PIN A0
#define CONVEYOR_IR_SENSOR_PIN 31 
#define GRIPPER_IR_PIN 30 
#define START_BUTTON_PIN 36
#define STOP_BUTTON_PIN 2
#define LIMIT_SWITCH_START_PIN 42
#define LIMIT_SWITCH_END_PIN 49

// --- Global Objects & Constants ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

#define SERVOMIN 246
#define SERVOMAX 492

// Servo Channels
#define BASE_SERVO 0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO 2
#define WRIST_SERVO 3
#define WRIST_ROLL_SERVO 4
#define PINCER_SERVO 5

#define PINCER_OPEN_ANGLE 90
#define PINCER_CLOSED_ANGLE 20
#define HEADER_BYTE 0xA5

File imgFile;
uint16_t fileCounter = 0;
volatile bool emergencyStopState = false;
volatile bool limitSwitchTripped = false;
bool systemRunning = false;

// Function Prototypes
void emergencyStop();
void checkLimitSwitches();
void displayMessage(String line1, String line2, int size, int color);
void moveServo(int servoNum, int angle, int maxAngle);
void pincerOpen();
void pincerClose();
void vacuumOn();
void vacuumOff();
uint16_t getNextFileIndex();
void receiveAndDisplayImage();
uint8_t readByteCam();
int getBinDistance();
void runWasteSortingSequence();

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial1.begin(19200); // Camera
  pwm.begin();
  pwm.setPWMFreq(60);
  tft.begin();
  tft.setRotation(0);

  if (!SD.begin(SD_CS)) {
    displayMessage("SD Card FAILED!", "", 2, ILI9341_RED);
    while (1);
  }
  fileCounter = getNextFileIndex();

  pinMode(CONVEYOR_IN1, OUTPUT);
  pinMode(CONVEYOR_IN2, OUTPUT);
  pinMode(CONVEYOR_ENA, OUTPUT);
  pinMode(VACUUM_IN1, OUTPUT);
  pinMode(VACUUM_IN2, OUTPUT);
  pinMode(VACUUM_ENB, OUTPUT);
  pinMode(CAM_TRIGGER_PIN, OUTPUT);
  pinMode(CONVEYOR_IR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(GRIPPER_IR_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);
  
  for (int i = LIMIT_SWITCH_START_PIN; i <= LIMIT_SWITCH_END_PIN; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), emergencyStop, FALLING);

  vacuumOff();
  digitalWrite(CAM_TRIGGER_PIN, LOW);
  displayMessage("System Initializing...", "Press START", 2, ILI9341_WHITE);
}

void loop() {
  if (emergencyStopState) return;
  if (digitalRead(START_BUTTON_PIN) == HIGH && !systemRunning) {
    delay(50);
    if (digitalRead(START_BUTTON_PIN) == HIGH) {
      systemRunning = true;
      runWasteSortingSequence();
      systemRunning = false;
      if (!emergencyStopState) {
        displayMessage("Sequence Complete.", "Press START.", 2, ILI9341_CYAN);
      }
    }
  }
}

void runWasteSortingSequence() {
  displayMessage("System Starting...", "", 2, ILI9341_GREEN);
  delay(500); if (emergencyStopState) return;

  // Check Bin
  int distance = getBinDistance();
  while (distance < 10) {
    if (emergencyStopState) return;
    displayMessage("BIN FULL!", "Waiting...", 2, ILI9341_RED);
    delay(1000);
    distance = getBinDistance();
  }

  // Start Conveyor
  displayMessage("Starting Conveyor...", "", 2, ILI9341_WHITE);
  digitalWrite(CONVEYOR_IN1, HIGH);
  digitalWrite(CONVEYOR_IN2, LOW);
  analogWrite(CONVEYOR_ENA, 200);

  // Wait for IR Trigger
  while (digitalRead(CONVEYOR_IR_SENSOR_PIN) == HIGH) {
    if (emergencyStopState) return; delay(10);
  }

  // Stop Conveyor & Capture
  analogWrite(CONVEYOR_ENA, 0);
  delay(250); if (emergencyStopState) return;
  receiveAndDisplayImage(); 
  if (emergencyStopState) return;

  // Pick Sequence
  displayMessage("Object: PLASTIC", "Starting Pick...", 2, ILI9341_CYAN);
  pincerOpen();
  moveServo(BASE_SERVO, 90, 180); delay(500);
  moveServo(SHOULDER_SERVO, 45, 180); delay(500);
  moveServo(ELBOW_SERVO, 20, 180); delay(500);
  moveServo(WRIST_SERVO, 10, 90); delay(500);
  moveServo(WRIST_ROLL_SERVO, 90, 180); delay(500);

  // Proximity Check
  if(digitalRead(GRIPPER_IR_PIN) == HIGH) {
     displayMessage("GRIP FAILED!", "No object.", 2, ILI9341_RED);
     delay(2000);
     return; // Add return to home logic here
  }

  // Grip
  vacuumOn();
  delay(500); 
  pincerClose();
  delay(500);

  // Place
  moveServo(BASE_SERVO, 150, 180); delay(500);
  moveServo(SHOULDER_SERVO, 60, 180); delay(500);
  moveServo(ELBOW_SERVO, 45, 180); delay(500);

  // Release
  pincerOpen();
  delay(300);
  vacuumOff();
  delay(500);

  // Home
  moveServo(BASE_SERVO, 90, 180);
  moveServo(SHOULDER_SERVO, 90, 180);
  moveServo(ELBOW_SERVO, 90, 180);
  moveServo(WRIST_SERVO, 45, 90);
  moveServo(WRIST_ROLL_SERVO, 90, 180);
  delay(500);
}

int getBinDistance() {
  if (emergencyStopState) return 999;
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 50000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void receiveAndDisplayImage() {
  // Simplified for brevity: Trigger camera, read bytes from Serial1, 
  // save to SD, display on TFT.
  // (Full code implementation from Appendix A goes here)
  digitalWrite(CAM_TRIGGER_PIN, HIGH); delay(50);
  digitalWrite(CAM_TRIGGER_PIN, LOW);
  // ... image reading logic ...
}

uint16_t getNextFileIndex() {
  // SD card logic to find next IMGxxx.BIN
  return 0; // Placeholder
}

void moveServo(int servoNum, int angle, int maxAngle) {
  if (emergencyStopState) return;
  checkLimitSwitches();
  if (emergencyStopState) return;
  int pulselen = map(angle, 0, maxAngle, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulselen);
}

void pincerOpen() { moveServo(PINCER_SERVO, PINCER_OPEN_ANGLE, 180); }
void pincerClose() { moveServo(PINCER_SERVO, PINCER_CLOSED_ANGLE, 180); }
void vacuumOn() { 
  digitalWrite(VACUUM_IN1, HIGH); digitalWrite(VACUUM_IN2, LOW); analogWrite(VACUUM_ENB, 255); 
}
void vacuumOff() { 
  digitalWrite(VACUUM_IN1, LOW); digitalWrite(VACUUM_IN2, LOW); analogWrite(VACUUM_ENB, 0); 
}

void checkLimitSwitches() {
  for (int i = LIMIT_SWITCH_START_PIN; i <= LIMIT_SWITCH_END_PIN; i++) {
    if (digitalRead(i) == LOW) { limitSwitchTripped = true; emergencyStop(); return; }
  }
}

void displayMessage(String line1, String line2, int size, int color) {
  if (emergencyStopState && line1 != "! JOINT !" && line1 != "! EMERGENCY !") return;
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(10, 20); tft.setTextColor(color); tft.setTextSize(size);
  tft.println(line1);
  if (line2 != "") { tft.setCursor(10, tft.getCursorY() + 10); tft.println(line2); }
}

void emergencyStop() {
  if (!emergencyStopState) {
    emergencyStopState = true;
    analogWrite(CONVEYOR_ENA, 0); vacuumOff();
    for (int i = 0; i < 16; i++) { pwm.setPWM(i, 0, 0); }
    tft.fillScreen(ILI9341_RED);
    tft.setCursor(20, 50); tft.setTextColor(ILI9341_WHITE); tft.setTextSize(3);
    tft.println("SYSTEM HALTED");
  }
}
