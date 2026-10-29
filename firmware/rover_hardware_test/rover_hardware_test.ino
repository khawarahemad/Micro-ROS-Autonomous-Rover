#include <Arduino.h>
#include <DHT.h>

/*
 ==============================================================================
 Micro-ROS Autonomous Rover - Hardware Diagnostic & Verification Firmware
 Centurion University - B.Tech CSE Robotics ROS with C++
 ==============================================================================
 Confirmed Pin Mapping:
 - Right Motor (L298N): IN1 = GPIO 18 (D18), IN2 = GPIO 19 (D19)
 - Left Motor  (L298N): IN3 = GPIO 26 (D26), IN4 = GPIO 27 (D27)
 - Ultrasonic: Trigger = GPIO 5 (D5), Echo = GPIO 4 (D4)
 - Environmental: DHT11 = GPIO 32, LDR = GPIO 25, IR = GPIO 33
 ==============================================================================
*/

#define DHTPIN        32
#define DHTTYPE       DHT11
#define LDR_PIN       25
#define IR_PIN        33
#define TRIG_PIN      5
#define ECHO_PIN      4

// Updated clean pins for Right Motor:
#define MOTOR_R_IN1   18  // D18
#define MOTOR_R_IN2   19  // D19
#define MOTOR_L_IN3   26  // D26
#define MOTOR_L_IN4   27  // D27

DHT dht(DHTPIN, DHTTYPE);

bool continuousMode = false;
unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 1000;

float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1.0;
  return (float)(duration * 0.0343 / 2.0);
}

void stopMotors() {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  digitalWrite(MOTOR_L_IN3, LOW);
  digitalWrite(MOTOR_L_IN4, LOW);
}

void leftMotorFwd(int ms = 0) {
  digitalWrite(MOTOR_L_IN3, HIGH);
  digitalWrite(MOTOR_L_IN4, LOW);
  if (ms > 0) { delay(ms); digitalWrite(MOTOR_L_IN3, LOW); digitalWrite(MOTOR_L_IN4, LOW); }
}

void leftMotorBwd(int ms = 0) {
  digitalWrite(MOTOR_L_IN3, LOW);
  digitalWrite(MOTOR_L_IN4, HIGH);
  if (ms > 0) { delay(ms); digitalWrite(MOTOR_L_IN3, LOW); digitalWrite(MOTOR_L_IN4, LOW); }
}

void rightMotorFwd(int ms = 0) {
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  if (ms > 0) { delay(ms); digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, LOW); }
}

void rightMotorBwd(int ms = 0) {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  if (ms > 0) { delay(ms); digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, LOW); }
}

void moveForward(int ms = 0) {
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  digitalWrite(MOTOR_L_IN3, HIGH);
  digitalWrite(MOTOR_L_IN4, LOW);
  if (ms > 0) { delay(ms); stopMotors(); }
}

void moveBackward(int ms = 0) {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  digitalWrite(MOTOR_L_IN3, LOW);
  digitalWrite(MOTOR_L_IN4, HIGH);
  if (ms > 0) { delay(ms); stopMotors(); }
}

void runWheelCalibrationSequence() {
  Serial.println(F("\n=============================================="));
  Serial.println(F(">>> STARTING WHEEL CALIBRATION SEQUENCE <<<"));
  Serial.println(F("=============================================="));

  Serial.println(F("Step 1: LEFT WHEEL FORWARD (1.5s)..."));
  leftMotorFwd(1500);
  delay(500);

  Serial.println(F("Step 2: RIGHT WHEEL FORWARD (1.5s)..."));
  rightMotorFwd(1500);
  delay(500);

  Serial.println(F("Step 3: LEFT WHEEL BACKWARD (1.5s)..."));
  leftMotorBwd(1500);
  delay(500);

  Serial.println(F("Step 4: RIGHT WHEEL BACKWARD (1.5s)..."));
  rightMotorBwd(1500);
  delay(500);

  Serial.println(F("Step 5: BOTH WHEELS FORWARD (1.5s)..."));
  moveForward(1500);
  delay(500);

  Serial.println(F("Step 6: BOTH WHEELS BACKWARD (1.5s)..."));
  moveBackward(1500);
  delay(500);

  Serial.println(F("Step 7: EMERGENCY STOP (ALL MOTORS OFF)..."));
  stopMotors();

  Serial.println(F("=============================================="));
  Serial.println(F(">>> SEQUENCE COMPLETE! ALL MOTORS STOPPED. <<<"));
  Serial.println(F("==============================================\n"));
}

void reportTelemetry() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  int light = digitalRead(LDR_PIN);
  int irObstacle = digitalRead(IR_PIN);
  float distance = readUltrasonicDistance();

  Serial.print(F("[TELEM] Temp: "));
  if (isnan(temp)) Serial.print(F("ERR"));
  else { Serial.print(temp, 1); Serial.print(F("C")); }

  Serial.print(F(" | Hum: "));
  if (isnan(hum)) Serial.print(F("ERR"));
  else { Serial.print(hum, 1); Serial.print(F("%")); }

  Serial.print(F(" | LDR: "));
  Serial.print(light == 1 ? F("BRIGHT(1)") : F("DARK(0)"));

  Serial.print(F(" | IR: "));
  Serial.print(irObstacle == 0 ? F("OBSTACLE!(0)") : F("CLEAR(1)"));

  Serial.print(F(" | Sonar(D5,D4): "));
  if (distance < 0) {
    Serial.println(F("OUT_OF_RANGE"));
  } else {
    Serial.printf("%.1f cm\n", distance);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  dht.begin();
  pinMode(LDR_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_L_IN3, OUTPUT);
  pinMode(MOTOR_L_IN4, OUTPUT);
  stopMotors();

  Serial.println(F("\n--- Rover Firmware Ready (Right: 18, 19 | Left: 26, 27) ---"));
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == '\r' || cmd == '\n' || cmd == ' ') return;

    Serial.printf(">> Received: '%c'\n", cmd);
    switch (cmd) {
      case 'w': runWheelCalibrationSequence(); break;
      case '1': Serial.println(F("Left Forward 1.5s...")); leftMotorFwd(1500); Serial.println(F("Done.")); break;
      case '2': Serial.println(F("Right Forward 1.5s...")); rightMotorFwd(1500); Serial.println(F("Done.")); break;
      case '3': Serial.println(F("Left Backward 1.5s...")); leftMotorBwd(1500); Serial.println(F("Done.")); break;
      case '4': Serial.println(F("Right Backward 1.5s...")); rightMotorBwd(1500); Serial.println(F("Done.")); break;
      case 'f': Serial.println(F("Both Forward 1.5s...")); moveForward(1500); Serial.println(F("Done.")); break;
      case 'b': Serial.println(F("Both Backward 1.5s...")); moveBackward(1500); Serial.println(F("Done.")); break;
      case 'x': stopMotors(); Serial.println(F("All Motors Stopped.")); break;
      case 's': reportTelemetry(); break;
      case 'c': continuousMode = !continuousMode; Serial.printf("Telemetry: %s\n", continuousMode ? "ON" : "OFF"); break;
      default: break;
    }
  }

  if (continuousMode && (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)) {
    lastTelemetryMs = millis();
    reportTelemetry();
  }
}
