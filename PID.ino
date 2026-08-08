
PID program 

#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// --- PID Control Variables ---
float Kp = 2.5; 
float Ki = 0.1; 
float Kd = 0.5;
float pError = 0, iTerm = 0;
const float I_LIMIT = 50.0; 

// --- Pin Definitions ---
const int enA = 25; const int in1 = 16; const int in2 = 17; // Exhaust Valve
const int enB = 26; const int in3 = 19; const int in4 = 18; // Inlet Valve
const int inputPin = 33;
const int springPin = 32;

// --- Objects ---
VL53L0X sensor;
MPU6050 imu(0x68);

// --- System Variables ---
float targetHeight = 0.0;
float currentHeight = 0.0;
const float DEADZONE = 4.0;   // 4mm Deadzone for stability
const unsigned long CYCLE_TIME = 143; // 7Hz Valve Pulsing
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL = 40; // 25Hz Refresh Rate

// --- Kalman Filter States ---
float QH = 0.5, RH = 2.0, PH = 1.0, XH = 0.0;

float kalmanHeight(float measurement) {
  PH = PH + QH;
  float K = PH / (PH + RH);
  XH = XH + K * (measurement - XH);
  PH = (1 - K) * PH;
  return XH;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  analogReadResolution(12);

  if (!sensor.init()) {
    Serial.println("VL53L0X Fail");
    while (1);
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
  imu.initialize();
}

void loop() {
  // 1. Listen for PID/Target updates from Python
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'T') targetHeight = val;
    else if (type == 'P') Kp = val;
    else if (type == 'I') Ki = val;
    else if (type == 'D') Kd = val;
    while(Serial.available() > 0) Serial.read(); // Clear buffer
  }

  // 2. Main Control Loop
  unsigned long now = millis();
  if (now - lastLoopTime >= LOOP_INTERVAL) {
    float dt = (now - lastLoopTime) / 1000.0;
    lastLoopTime = now;

    // Read Sensors
    uint16_t rawDist = sensor.readRangeContinuousMillimeters();
    currentHeight = kalmanHeight(rawDist);
    int pInRaw = analogRead(inputPin);
    int pSpRaw = analogRead(springPin);
    
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 3. PID Math
    float error = targetHeight - currentHeight;
    int dutyCycle = 0;
    int statusFlag = 0; // 0:Idle, 1:Filling, 2:Draining

    if (abs(error) > DEADZONE) {
      iTerm += error * dt;
      iTerm = constrain(iTerm, -I_LIMIT, I_LIMIT);
      float dTerm = (error - pError) / dt;
      
      float output = (Kp * error) + (Ki * iTerm) + (Kd * dTerm);
      dutyCycle = constrain(abs(output), 20, 100); 
      statusFlag = (output > 0) ? 1 : 2; 
      pError = error;
    } else {
      iTerm = 0; // Reset integral inside deadzone to prevent "jump" when leaving
      pError = 0;
    }

    // Command Valves
    runBinaryPulsing(statusFlag, dutyCycle);

    // CSV Telemetry output
    Serial.print(now); Serial.print(",");
    Serial.print(targetHeight, 0); Serial.print(",");
    Serial.print(currentHeight, 1); Serial.print(",");
    Serial.print(statusFlag == 2 ? -dutyCycle : dutyCycle); Serial.print(",");
    Serial.print(pInRaw); Serial.print(",");
    Serial.print(pSpRaw); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }
}

void runBinaryPulsing(int status, int dc) {
  unsigned long elapsed = millis() % CYCLE_TIME;
  unsigned long onTime = (dc * CYCLE_TIME) / 100;
  bool valveOpen = (elapsed < onTime);

  if (status == 1 && valveOpen) { // Filling
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW); digitalWrite(enB, HIGH);
    digitalWrite(enA, LOW);
  } 
  else if (status == 2 && valveOpen) { // Draining
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(enA, HIGH);
    digitalWrite(enB, LOW);
  } 
  else { // Off
    digitalWrite(enA, LOW); digitalWrite(enB, LOW);
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  }
}
