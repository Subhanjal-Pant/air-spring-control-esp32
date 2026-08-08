#include <Wire.h>
#include <VL53L0X.h>

// --- Pin Definitions ---
// Motor 1 (Exhaust Solenoid)
const int enA = 25;
const int in1 = 16;
const int in2 = 17;


// Motor 2 (Inlet Solenoid)
const int enB = 26;
const int in3 = 19;
const int in4 = 18;


// --- Sensor & Control Variables ---
VL53L0X sensor;
float targetHeight = 0.0;    // Desired height in mm
float currentHeight = 0.0;   // Filtered distance in mm
const float tolerance = 20.0; // 20mm deadband

// --- Kalman Filter Variables ---
float Q = 0.5; 
float R = 2.0; 
float P = 1.0; 
float X = 0.0; 

float kalmanFilter(float measurement) {
  P = P + Q;
  float K = P / (P + R);
  X = X + K * (measurement - X);
  P = (1 - K) * P;
  return X;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Solenoid Control Pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Start with valves closed
  stopExhaust();
  stopInlet();

  // Initialize ToF Sensor
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("VL53L0X not detected! Check wiring.");
    while (1);
  }
  
  // High accuracy mode (200ms)
  sensor.setMeasurementTimingBudget(200000); 
  sensor.startContinuous();

  Serial.println("\n--- Bang-Bang Height Control Initialized ---");
  Serial.println("Type the target height (mm) and press ENTER.");
}

void loop() {
  // 1. Check for User Input (Target Height)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    float newTarget = input.toFloat();
    if (newTarget >= 0) {
      targetHeight = newTarget;
      Serial.print("\n>>> TARGET UPDATED TO: ");
      Serial.print(targetHeight);
      Serial.println(" mm <<<\n");
    }
  }

  // 2. Read ToF Sensor
  uint16_t raw = sensor.readRangeContinuousMillimeters();
  
  if (sensor.timeoutOccurred()) {
    Serial.println("SENSOR TIMEOUT");
    stopInlet();
    stopExhaust();
  } else {
    // Apply Kalman Filter
    currentHeight = kalmanFilter(raw);
    
    // 3. Bang-Bang Control Logic & Status Messaging
    String statusMsg = "HOLDING";

    if (targetHeight > 0) {
      float error = targetHeight - currentHeight;

      if (error > tolerance) {
        openInlet();
        stopExhaust();
        statusMsg = "FILLING (INLET ON)";
      } 
      else if (error < -tolerance) {
        openExhaust();
        stopInlet();
        statusMsg = "DRAINING (EXHAUST ON)";
      } 
      else {
        stopInlet();
        stopExhaust();
        statusMsg = "TARGET REACHED (VALVES CLOSED)";
      }
    }

    // 4. Continuous Serial Output
    // Format: [Target] [Current] [Status]
    Serial.print("Target: ");
    Serial.print(targetHeight, 0);
    Serial.print("mm | Current: ");
    Serial.print(currentHeight, 1);
    Serial.print("mm | Status: ");
    Serial.println(statusMsg);
  }

  delay(50); // High frequency output (approx 20Hz)
}

// --- Valve Control Functions ---

void openExhaust() {
  digitalWrite(enA, HIGH);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void stopExhaust() {
  digitalWrite(enA, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void openInlet() {
  digitalWrite(enB, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopInlet() {
  digitalWrite(enB, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

