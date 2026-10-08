/*
  Active Radar Haptic Trackpad (Sticky Tracking Update)
  Baud Rate: 9600
  Servo 1 (Left/Y):  Shield Pin 10 | Sensor 1: Trig A2, Echo A3
  Servo 2 (Bottom/X): Shield Pin 9 | Sensor 2: Trig A0, Echo A1
*/

#include <Servo.h>

const float PLANE_WIDTH = 32.0; 
const float PLANE_HEIGHT = 18.0;
const float R_ARM = 2.5; 
const float Z_DIFF = 0.5; 

const int MIN_SERVO_ANGLE = 35;
const int MAX_SERVO_ANGLE = 145;
const int MAX_DIST_CM = 45;

// INCREASED: Gives the sensor twice as long to listen for an echo
const int TIMEOUT_US = 6000; 

const int TRG1 = A2; const int ECH1 = A3; const int SRV1 = 10; 
const int TRG2 = A0; const int ECH2 = A1; const int SRV2 = 9;  

Servo servo1;
Servo servo2;

float smoothX = 16.0;
float smoothY = 9.0;
const float EMA_ALPHA = 0.4; 

float current_A1 = 90.0;
float current_A2 = 90.0;
int lastWrittenA1 = -1;
int lastWrittenA2 = -1;

bool isLockedOn = false;

// --- NEW STATEFUL SWEEP & TOLERANCE GLOBALS ---
const int MISS_LIMIT = 6;
int missCount = 0;
float sweepPos = 90.0; 
int sweepDir = 1;

void setup() {
  Serial.begin(9600); 
  
  pinMode(TRG1, OUTPUT); pinMode(ECH1, INPUT);
  pinMode(TRG2, OUTPUT); pinMode(ECH2, INPUT);
  
  servo1.attach(SRV1);
  servo1.write(90);
  delay(500); 
  
  servo2.attach(SRV2);
  servo2.write(90);
  delay(500);

  lastWrittenA1 = 90;
  lastWrittenA2 = 90;
}

float getRawDistance(int trg, int ech) {
  digitalWrite(trg, LOW); delayMicroseconds(2);
  digitalWrite(trg, HIGH); delayMicroseconds(10);
  digitalWrite(trg, LOW);
  
  long duration = pulseIn(ech, HIGH, TIMEOUT_US);
  if (duration == 0) return -1.0; 
  return (duration * 0.0343) / 2.0;
}

void loop() {
  float raw_d1 = getRawDistance(TRG1, ECH1);
  delay(20); 
  float raw_d2 = getRawDistance(TRG2, ECH2);
  delay(20);

  bool s1_sees = (raw_d1 > 2 && raw_d1 < MAX_DIST_CM);
  bool s2_sees = (raw_d2 > 2 && raw_d2 < MAX_DIST_CM);

  // --- DEBUG OUTPUT (Ignored by Python) ---
  Serial.print("# d1="); Serial.print(raw_d1); 
  Serial.print(" d2="); Serial.print(raw_d2);
  Serial.print(" a1="); Serial.print(current_A1); 
  Serial.print(" a2="); Serial.println(current_A2);

  if (s1_sees || s2_sees) {
    // FINGER DETECTED: Reset the miss counter instantly
    missCount = 0;
    
    float rad1 = (current_A1 - 90.0) * PI / 180.0;
    float rad2 = current_A2 * PI / 180.0;
    
    float s1_x = 0.0 + (R_ARM * cos(rad1));
    float s1_y = 9.0 + (R_ARM * sin(rad1));
    
    float s2_x = 16.0 + (R_ARM * cos(rad2));
    float s2_y = 0.0 + (R_ARM * sin(rad2));

    float val1 = (raw_d1 * raw_d1) - (Z_DIFF * Z_DIFF);
    float flat_d1 = (val1 > 0) ? sqrt(val1) : 0.0;
    
    float val2 = (raw_d2 * raw_d2) - (Z_DIFF * Z_DIFF);
    float flat_d2 = (val2 > 0) ? sqrt(val2) : 0.0;

    float calcX = 0; float calcY = 0; int activeSensors = 0;

    if (s1_sees) {
      calcX += s1_x + (flat_d1 * cos(rad1));
      calcY += s1_y + (flat_d1 * sin(rad1));
      activeSensors++;
    }
    if (s2_sees) {
      calcX += s2_x + (flat_d2 * cos(rad2));
      calcY += s2_y + (flat_d2 * sin(rad2));
      activeSensors++;
    }

    float rawX = calcX / activeSensors;
    float rawY = calcY / activeSensors;

    if (!isLockedOn) {
      smoothX = rawX; 
      smoothY = rawY; 
      isLockedOn = true;
    } else {
      smoothX = (EMA_ALPHA * rawX) + ((1.0 - EMA_ALPHA) * smoothX);
      smoothY = (EMA_ALPHA * rawY) + ((1.0 - EMA_ALPHA) * smoothY);
    }
    
    smoothX = constrain(smoothX, 0, PLANE_WIDTH);
    smoothY = constrain(smoothY, 0, PLANE_HEIGHT);

    current_A1 = atan2(smoothY - 9.0, smoothX - 0.0) * 180.0 / PI + 90.0;
    current_A2 = atan2(smoothY - 0.0, smoothX - 16.0) * 180.0 / PI;

    Serial.print(smoothX, 2); 
    Serial.print(","); 
    Serial.println(smoothY, 2);

  } else if (isLockedOn && ++missCount < MISS_LIMIT) {
    // BRIEF DROPOUT: Hand disappeared for a split second. 
    // Hold physical position and keep transmitting the last known good coordinate.
    Serial.print(smoothX, 2); 
    Serial.print(","); 
    Serial.println(smoothY, 2);

  } else {
    // TARGET LOST COMPLETELY: Resume smooth sweeping from current position
    if (isLockedOn) { 
      sweepPos = constrain(current_A1, 45, 135); 
      isLockedOn = false; 
    }
    
    sweepPos += sweepDir * 2.0;
    if (sweepPos >= 135) sweepDir = -1;
    if (sweepPos <= 45)  sweepDir = 1;
    
    current_A1 = sweepPos;
    current_A2 = sweepPos;
    Serial.println("OUT");
  }

  // Actuate Servos
  current_A1 = constrain(current_A1, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  current_A2 = constrain(current_A2, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

  int targetA1 = (int)round(current_A1);
  int targetA2 = (int)round(current_A2);

  if (abs(targetA1 - lastWrittenA1) >= 2) {
    servo1.write(targetA1);
    lastWrittenA1 = targetA1;
  }
  if (abs(targetA2 - lastWrittenA2) >= 2) {
    servo2.write(targetA2);
    lastWrittenA2 = targetA2;
  }
}