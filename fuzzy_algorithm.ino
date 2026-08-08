Fuzzy new algo- actual working 

#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// --- Pin Definitions ---
const int enA = 25; const int in1 = 16; const int in2 = 17; // Exhaust Valve
const int enB = 26; const int in3 = 19; const int in4 = 18; // Inlet Valve
const int inputPin = 33;
const int springPin = 32;

// --- Objects ---
VL53L0X sensor;
MPU6050 imu(0x68);

// --- System Variables ---
float targetHeight = 130.0;
float currentHeight = 0.0;
const float DEADZONE = 4.0;           // Kept at 4mm as requested
const unsigned long CYCLE_TIME = 143; // 7Hz Valve Pulsing
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL = 40; // 25Hz Refresh Rate

// Global control variables
int sysDutyCycle = 0;
int sysStatusFlag = 0; // 0:Idle, 1:Filling, 2:Draining

// --- Sensor Calibration ---
const float CAL_SLOPE = 0.001845;
const float CAL_OFFSET = -1.1914;

// --- Kalman Filter States ---
// RH increased to 8.0 to reduce "tof error" jitter and valve vibration
float QH = 0.5, RH = 8.0, PH = 1.0, XH = 0.0;

float kalmanHeight(float measurement) {
  PH = PH + QH;
  float K = PH / (PH + RH);
  XH = XH + K * (measurement - XH);
  PH = (1 - K) * PH;
  return XH;
}

// --- Fuzzy Logic Helper: Triangular Membership Function ---
float trimf(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x == b) return 1.0;
  if (x > a && x < b) return (x - a) / (b - a);
  if (x > b && x < c) return (c - x) / (c - b);
  return 0.0;
}

// --- Fuzzy Logic Engine ---
// ASYMMETRIC SINGLETONS: Inflation is much faster than exhausting due to tank pressure.
// Inlet power is lower to prevent the overshoot at 190mm.
const float OCV = -100; // Exhaust Very Big
const float OCB = -55;  // Exhaust Big
const float OCS = -25;  // Exhaust Small
const float IDLE = 0;
const float OIS = 12;   // Inlet Small (Reduced for soft touch near target)
const float OIB = 35;   // Inlet Big (Toned down)
const float OIV = 80;   // Inlet Very Big (Limited to 80% to maintain control)

// Rule Base Matrix [Error][DeltaP]
const float ruleBase[7][7] = {
  {OCB,  OCV,  OCV,  OCV,  OCV,  OCV,  OCV}, // NVB 
  {OCS,  OCB,  OCB,  OCB,  OCV,  OCV,  OCV}, // NB
  {OCS,  OCS,  OCS,  OCS,  OCB,  OCB,  OCB}, // NS
  {IDLE, IDLE, IDLE, IDLE, IDLE, IDLE, IDLE},// ZO 
  {OIS,  OIS,  OIS,  OIS,  OIB,  OIB,  OIB}, // PS
  {OIS,  OIB,  OIB,  OIB,  OIV,  OIV,  OIV}, // PB
  {OIB,  OIV,  OIV,  OIV,  OIV,  OIV,  OIV}  // PVB 
};

int computeFuzzy(float error, float dp) {
  // 1. Fuzzification for Height Error (e)
  float mu_e[7];
  mu_e[0] = trimf(error, -50, -40, -25); // NVB
  mu_e[1] = trimf(error, -35, -25, -12); // NB
  mu_e[2] = trimf(error, -18, -10, -4);  // NS
  mu_e[3] = trimf(error, -8, 0, 8);      // ZO
  mu_e[4] = trimf(error, 4, 10, 18);     // PS
  mu_e[5] = trimf(error, 12, 25, 35);    // PB
  mu_e[6] = trimf(error, 25, 40, 50);    // PVB

  // 2. Fuzzification for Pressure Difference (dp)
  float mu_dp[7];
  if (error < 0) {
    // EXHAUSTING: Spring Pressure (Support for negative pressure -0.3 to 1.2 bar)
    mu_dp[0] = trimf(dp, -0.5, -0.3, 0.0); 
    mu_dp[1] = trimf(dp, -0.3, 0.0, 0.2);  
    mu_dp[2] = trimf(dp, 0.0, 0.2, 0.4);  
    mu_dp[3] = trimf(dp, 0.2, 0.4, 0.7);  
    mu_dp[4] = trimf(dp, 0.4, 0.7, 0.9);  
    mu_dp[5] = trimf(dp, 0.7, 0.9, 1.2);  
    mu_dp[6] = trimf(dp, 0.9, 1.2, 1.5);  
  } else {
    // INFLATING: Differential Pressure (Tank - Spring: 0.0 to 4.0 bar)
    mu_dp[0] = trimf(dp, -0.5, 0.0, 0.5); 
    mu_dp[1] = trimf(dp, 0.0, 0.5, 1.2);  
    mu_dp[2] = trimf(dp, 0.5, 1.2, 1.8);  
    mu_dp[3] = trimf(dp, 1.2, 1.8, 2.4);  // Typical 190mm range
    mu_dp[4] = trimf(dp, 1.8, 2.4, 3.0);  
    mu_dp[5] = trimf(dp, 2.4, 3.0, 3.6);  
    mu_dp[6] = trimf(dp, 3.0, 3.6, 4.5);  
  }

  // 3. Inference and Defuzzification
  float num = 0.0;
  float den = 0.0;
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      float weight = min(mu_e[i], mu_dp[j]); 
      if (weight > 0) {
        num += weight * ruleBase[i][j];
        den += weight;
      }
    }
  }

  if (den == 0) return 0;
  return (int)(num / den);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); 
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
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'T') targetHeight = val;
    while(Serial.available() > 0) Serial.read(); 
  }

  unsigned long now = millis();

  if (now - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = now;

    uint16_t rawDist = sensor.readRangeContinuousMillimeters();
    currentHeight = kalmanHeight(rawDist);
    int pInRaw = analogRead(inputPin);
    int pSpRaw = analogRead(springPin);
    
    float pInletBar = (pInRaw * CAL_SLOPE) + CAL_OFFSET;
    float pSpringBar = (pSpRaw * CAL_SLOPE) + CAL_OFFSET;
    
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float error = targetHeight - currentHeight;
    float deltaP = 0.0;
    
    if (error > DEADZONE) {
      deltaP = pInletBar - pSpringBar; 
    } else if (error < -DEADZONE) {
      deltaP = pSpringBar; 
    }

    if (abs(error) > DEADZONE) {
      float constrainedError = constrain(error, -45.0, 45.0);
      float constrainedDp;
      if (error < 0) {
        constrainedDp = constrain(deltaP, -0.3, 1.2); // Support negative pressure
      } else {
        constrainedDp = constrain(deltaP, 0.0, 4.0); 
      }

      int fuzzyOutput = computeFuzzy(constrainedError, constrainedDp);
      
      // COASTING: Turn off early to let momentum carry into the deadzone
      if (fuzzyOutput > 10) { 
        sysStatusFlag = 1;
        sysDutyCycle = constrain(fuzzyOutput, 15, 100);
      } else if (fuzzyOutput < -10) {
        sysStatusFlag = 2;
        sysDutyCycle = constrain(abs(fuzzyOutput), 15, 100);
      } else {
        sysStatusFlag = 0;
        sysDutyCycle = 0;
      }
    } else {
      sysStatusFlag = 0;
      sysDutyCycle = 0;
    }

    // CSV Telemetry output (Matches your original format exactly)
    Serial.print(now); Serial.print(",");
    Serial.print(targetHeight, 0); Serial.print(",");
    Serial.print(currentHeight, 1); Serial.print(",");
    Serial.print(sysStatusFlag == 2 ? -sysDutyCycle : sysDutyCycle); Serial.print(",");
    Serial.print(pInRaw); Serial.print(",");
    Serial.print(pSpRaw); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }

  runBinaryPulsing(sysStatusFlag, sysDutyCycle);
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

