#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>

/*
 ==============================================================================
 Micro-ROS Autonomous Environmental Rover - PWM Speed Control & Smart Nav
 Centurion University - B.Tech CSE Robotics ROS with C++
 ==============================================================================
 Features:
 1. PWM Motor Speed Control (0-255) via ESP32 LEDC hardware
 2. Web Speed Slider (Accelerator) in Manual Mode
 3. Smart Autonomous Mode: Distance-proportional speed + path change logic
 4. Autonomous AP Mode ("MicroROS-Rover") + optional Station Mode
 5. Embedded Web Dashboard with Real-time Telemetry
 6. Environmental Mapping: DHT11, LDR, IR, HC-SR04
 ==============================================================================
*/

// ======================= PIN CONFIGURATION =======================
// Environmental Sensors
#define DHTPIN        32      // DHT11 Data Pin
#define DHTTYPE       DHT11   // DHT11 sensor model
#define LDR_PIN       25      // LDR Module Digital Out (DO)

// Obstacle Avoidance Sensors
#define IR_PIN        33      // IR Proximity Module OUT (0=obstacle, 1=clear)
#define TRIG_PIN      5       // HC-SR04 Ultrasonic Trigger Pin (D5)
#define ECHO_PIN      39      // HC-SR04 Ultrasonic Echo Pin (GPIO 39 / VN)

// Motor Driver (L298N)
#define MOTOR_R_IN1   18      // Right Motor IN1 (D18)
#define MOTOR_R_IN2   19      // Right Motor IN2 (D19)
#define MOTOR_L_IN3   26      // Left Motor IN3 (D26)
#define MOTOR_L_IN4   27      // Left Motor IN4 (D27)

// ======================= PWM CONFIGURATION =======================
// ESP32 LEDC PWM Channels (0-15 available)
#define PWM_CH_R1  0
#define PWM_CH_R2  1
#define PWM_CH_L3  2
#define PWM_CH_L4  3
#define PWM_FREQ   1000     // 1 kHz PWM frequency
#define PWM_RES    8        // 8-bit resolution (0-255)

// ======================= WI-FI CREDENTIALS =======================
const char* AP_SSID = "MicroROS-Rover";
const char* AP_PASS = "rover1234";
const char* STA_SSID = "YOUR_WIFI_NAME";
const char* STA_PASS = "YOUR_WIFI_PASSWORD";

WebServer server(80);
DHT dht(DHTPIN, DHTTYPE);

// Rover State
enum RoverMode { MODE_MANUAL, MODE_AUTONOMOUS };
RoverMode currentMode = MODE_MANUAL;
String currentAction = "STOPPED";
unsigned long lastActionTime = 0;
const unsigned long ACTION_TIMEOUT_MS = 600;

// Speed Control
int manualSpeed = 120;       // Default manual speed (0-255), ~47% duty
int currentPWM = 0;          // Actual PWM being applied right now
const int MIN_PWM = 60;      // Below this, motors stall (not enough torque)
const int MAX_PWM = 255;     // Full throttle

// Autonomous Speed Zones (distance in cm)
const float ZONE_FULL    = 60.0;   // >60cm: cruise at full speed
const float ZONE_MEDIUM  = 35.0;   // 35-60cm: medium speed approach
const float ZONE_SLOW    = 18.0;   // 18-35cm: slow creep
const float ZONE_STOP    = 10.0;   // <10cm: emergency stop & reroute
const int   AUTO_FULL    = 180;
const int   AUTO_MEDIUM  = 130;
const int   AUTO_SLOW    = 80;

// Telemetry Cache
float currentTemp = 0.0;
float currentHum = 0.0;
int currentLight = 0;
int currentIR = 1;
float currentDistance = -1.0;
bool obstacleEmergency = false;
unsigned long lastSensorReadMs = 0;

// Autonomous Navigation State
enum AutoState { AUTO_CRUISE, AUTO_BACKUP, AUTO_SCAN_LEFT, AUTO_SCAN_RIGHT, AUTO_DECIDE };
AutoState autoState = AUTO_CRUISE;
unsigned long autoStateStartMs = 0;
float scanLeftDist = 0;
float scanRightDist = 0;

// ======================= PWM MOTOR FUNCTIONS =======================
void setupMotorPWM() {
  // Attach LEDC channels to motor pins
  ledcAttach(MOTOR_R_IN1, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_R_IN2, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_L_IN3, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_L_IN4, PWM_FREQ, PWM_RES);
}

// Set all 4 motor pins with individual PWM values
void setMotors(int r1, int r2, int l3, int l4) {
  ledcWrite(MOTOR_R_IN1, constrain(r1, 0, 255));
  ledcWrite(MOTOR_R_IN2, constrain(r2, 0, 255));
  ledcWrite(MOTOR_L_IN3, constrain(l3, 0, 255));
  ledcWrite(MOTOR_L_IN4, constrain(l4, 0, 255));
}

void stopRover() {
  setMotors(0, 0, 0, 0);
  currentPWM = 0;
  currentAction = "STOPPED";
}

void driveForward(int speed) {
  if (obstacleEmergency && currentMode == MODE_MANUAL) {
    stopRover();
    currentAction = "BLOCKED (<10cm)";
    return;
  }
  int spd = constrain(speed, MIN_PWM, MAX_PWM);
  setMotors(spd, 0, spd, 0);
  currentPWM = spd;
  currentAction = "FORWARD";
  lastActionTime = millis();
}

void driveBackward(int speed) {
  int spd = constrain(speed, MIN_PWM, MAX_PWM);
  setMotors(0, spd, 0, spd);
  currentPWM = spd;
  currentAction = "BACKWARD";
  lastActionTime = millis();
}

void spinLeft(int speed) {
  int spd = constrain(speed, MIN_PWM, MAX_PWM);
  setMotors(spd, 0, 0, spd);
  currentPWM = spd;
  currentAction = "TURNING LEFT";
  lastActionTime = millis();
}

void spinRight(int speed) {
  int spd = constrain(speed, MIN_PWM, MAX_PWM);
  setMotors(0, spd, spd, 0);
  currentPWM = spd;
  currentAction = "TURNING RIGHT";
  lastActionTime = millis();
}

// ======================= SENSOR FUNCTIONS =======================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  if (duration == 0) return -1.0;
  return (float)(duration * 0.0343 / 2.0);
}

void updateSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) currentTemp = t;
  if (!isnan(h)) currentHum = h;

  currentLight = digitalRead(LDR_PIN);
  currentIR = digitalRead(IR_PIN);
  float dist = readUltrasonic();
  if (dist > 0) currentDistance = dist;
  else currentDistance = -1.0;

  // Emergency: <10cm or IR triggered
  if ((currentDistance > 0 && currentDistance < ZONE_STOP) || currentIR == 0) {
    obstacleEmergency = true;
    if (currentMode == MODE_MANUAL && (currentAction == "FORWARD")) {
      stopRover();
      currentAction = "EMERGENCY STOP";
    }
  } else {
    obstacleEmergency = false;
  }
}

// ======================= SMART AUTONOMOUS NAVIGATION =======================
// State-machine based: Cruise -> Detect -> Backup -> Scan Left -> Scan Right -> Decide -> Cruise
void handleAutonomousBehavior() {
  unsigned long elapsed = millis() - autoStateStartMs;

  switch (autoState) {

    case AUTO_CRUISE: {
      if (currentDistance < 0) {
        // No echo - cautiously cruise at medium speed
        driveForward(AUTO_MEDIUM);
        return;
      }

      if (currentDistance > ZONE_FULL) {
        // Open road - full speed
        driveForward(AUTO_FULL);
      } else if (currentDistance > ZONE_MEDIUM) {
        // Approaching something - medium speed
        driveForward(AUTO_MEDIUM);
      } else if (currentDistance > ZONE_SLOW) {
        // Getting close - slow creep
        driveForward(AUTO_SLOW);
      } else {
        // Too close! Start evasive maneuver
        stopRover();
        currentAction = "OBSTACLE DETECTED";
        autoState = AUTO_BACKUP;
        autoStateStartMs = millis();
      }
      break;
    }

    case AUTO_BACKUP: {
      driveBackward(AUTO_SLOW);
      currentAction = "BACKING UP";
      if (elapsed > 350) {
        stopRover();
        autoState = AUTO_SCAN_LEFT;
        autoStateStartMs = millis();
      }
      break;
    }

    case AUTO_SCAN_LEFT: {
      spinLeft(AUTO_SLOW);
      currentAction = "SCANNING LEFT";
      if (elapsed > 400) {
        stopRover();
        scanLeftDist = readUltrasonic();
        if (scanLeftDist < 0) scanLeftDist = 999; // No echo = open space
        autoState = AUTO_SCAN_RIGHT;
        autoStateStartMs = millis();
      }
      break;
    }

    case AUTO_SCAN_RIGHT: {
      spinRight(AUTO_SLOW);
      currentAction = "SCANNING RIGHT";
      if (elapsed > 800) { // Spin past center to right
        stopRover();
        scanRightDist = readUltrasonic();
        if (scanRightDist < 0) scanRightDist = 999;
        autoState = AUTO_DECIDE;
        autoStateStartMs = millis();
      }
      break;
    }

    case AUTO_DECIDE: {
      // Pick the direction with more open space
      if (scanLeftDist > scanRightDist && scanLeftDist > ZONE_SLOW) {
        // Left is more open - spin back to left
        spinLeft(AUTO_MEDIUM);
        currentAction = "REROUTING LEFT";
      } else if (scanRightDist > ZONE_SLOW) {
        // Right is more open or equal - stay right (we're already facing right)
        currentAction = "REROUTING RIGHT";
        // Already pointing right from the scan, just cruise
      } else {
        // Both sides blocked - do a full 180 U-turn
        spinRight(AUTO_MEDIUM);
        currentAction = "U-TURN";
      }

      if (elapsed > 500) {
        stopRover();
        autoState = AUTO_CRUISE;
        autoStateStartMs = millis();
      }
      break;
    }
  }
}

// ======================= WEB SERVER HANDLERS =======================
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>Micro-ROS Rover Control</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0f172a; color: #f8fafc; text-align: center; padding: 12px; }
    header { margin-bottom: 14px; border-bottom: 1px solid #334155; padding-bottom: 10px; }
    h1 { font-size: 1.3rem; color: #38bdf8; }

    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(100px, 1fr)); gap: 8px; margin-bottom: 14px; }
    .card { background: #1e293b; border-radius: 10px; padding: 10px; border: 1px solid #334155; }
    .card-title { font-size: 0.65rem; color: #94a3b8; text-transform: uppercase; margin-bottom: 2px; }
    .card-val { font-size: 1.2rem; font-weight: bold; color: #38bdf8; }
    .alert { color: #ef4444 !important; animation: flash 1s infinite; }
    @keyframes flash { 0%,100%{opacity:1}50%{opacity:0.4} }

    .status-bar { background: #1e293b; border-radius: 8px; padding: 8px; margin-bottom: 12px; border: 1px solid #334155; font-size: 0.85rem; }
    .hl { color: #22c55e; font-weight: bold; }

    /* Speed Slider */
    .speed-box { background: #1e293b; border-radius: 10px; padding: 12px; border: 1px solid #334155; margin-bottom: 14px; }
    .speed-label { font-size: 0.75rem; color: #94a3b8; text-transform: uppercase; margin-bottom: 6px; }
    .speed-val { font-size: 1.6rem; font-weight: 800; color: #38bdf8; margin-bottom: 6px; }
    input[type=range] { -webkit-appearance: none; width: 90%; height: 8px; border-radius: 4px; background: #334155; outline: none; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 26px; height: 26px; border-radius: 50%; background: #38bdf8; cursor: pointer; }
    .speed-bar-bg { width: 90%; height: 6px; background: #334155; border-radius: 3px; margin: 6px auto 0; position: relative; }
    .speed-bar-fill { height: 100%; border-radius: 3px; background: linear-gradient(90deg, #22c55e, #f59e0b, #ef4444); transition: width 0.15s; }

    /* Controls */
    .controls { display: flex; flex-direction: column; align-items: center; gap: 7px; margin: 0 auto; max-width: 260px; }
    .d-row { display: flex; gap: 7px; justify-content: center; }
    .btn { background: #334155; color: #fff; border: 1px solid #475569; width: 68px; height: 58px; border-radius: 12px; font-size: 1.4rem; cursor: pointer; user-select: none; display: flex; align-items: center; justify-content: center; transition: 0.1s; }
    .btn:active,.btn.active { background: #0284c7; border-color: #38bdf8; transform: scale(0.94); }
    .btn-stop { background: #991b1b; }
    .btn-stop:active { background: #dc2626; }

    .mode-toggle { margin-top: 12px; display: flex; justify-content: center; gap: 8px; }
    .mode-btn { background: #1e293b; border: 1px solid #475569; color: #94a3b8; padding: 7px 14px; border-radius: 18px; font-weight: bold; cursor: pointer; font-size: 0.8rem; }
    .mode-btn.active { background: #0284c7; color: #fff; border-color: #38bdf8; }

    .hint { font-size: 0.7rem; color: #64748b; margin-top: 10px; }
  </style>
</head>
<body>
  <header><h1>🤖 Micro-ROS Rover</h1></header>

  <div class="grid">
    <div class="card"><div class="card-title">Temp</div><div class="card-val" id="temp">--</div></div>
    <div class="card"><div class="card-title">Humidity</div><div class="card-val" id="hum">--</div></div>
    <div class="card"><div class="card-title">Sonar</div><div class="card-val" id="sonar">--</div></div>
    <div class="card"><div class="card-title">IR</div><div class="card-val" id="ir">--</div></div>
    <div class="card"><div class="card-title">Light</div><div class="card-val" id="ldr">--</div></div>
    <div class="card"><div class="card-title">PWM</div><div class="card-val" id="pwm">0</div></div>
  </div>

  <div class="status-bar">Status: <span id="action" class="hl">STOPPED</span></div>

  <div class="speed-box">
    <div class="speed-label">Speed / Accelerator</div>
    <div class="speed-val" id="spd-display">120</div>
    <input type="range" id="spd-slider" min="60" max="255" value="120"
           oninput="onSpeedChange(this.value)">
    <div class="speed-bar-bg"><div class="speed-bar-fill" id="spd-bar" style="width:47%"></div></div>
  </div>

  <div class="controls">
    <div class="d-row"><button class="btn" id="bf" onpointerdown="drive('f')" onpointerup="drive('x')">▲</button></div>
    <div class="d-row">
      <button class="btn" id="bl" onpointerdown="drive('l')" onpointerup="drive('x')">◀</button>
      <button class="btn btn-stop" onclick="drive('x')">⏹</button>
      <button class="btn" id="br" onpointerdown="drive('r')" onpointerup="drive('x')">▶</button>
    </div>
    <div class="d-row"><button class="btn" id="bb" onpointerdown="drive('b')" onpointerup="drive('x')">▼</button></div>
  </div>

  <div class="mode-toggle">
    <button class="mode-btn active" id="m-man" onclick="setMode('manual')">Manual</button>
    <button class="mode-btn" id="m-auto" onclick="setMode('auto')">Auto Navigate</button>
  </div>

  <div class="hint">WASD / Arrows to Drive &middot; Spacebar = Brake &middot; Slider = Speed</div>

  <script>
    let curSpeed = 120;

    function drive(d) { fetch('/drive?dir='+d+'&spd='+curSpeed).catch(()=>{}); }
    function setMode(m) {
      fetch('/mode?m='+m).then(()=>{
        document.getElementById('m-man').classList.toggle('active', m==='manual');
        document.getElementById('m-auto').classList.toggle('active', m==='auto');
      }).catch(()=>{});
    }
    function onSpeedChange(v) {
      curSpeed = parseInt(v);
      document.getElementById('spd-display').innerText = curSpeed;
      let pct = ((curSpeed - 60) / (255 - 60)) * 100;
      document.getElementById('spd-bar').style.width = pct + '%';
      fetch('/speed?v=' + curSpeed).catch(()=>{});
    }

    let ak = null;
    window.addEventListener('keydown', e => {
      if(ak) return;
      const k = e.key.toLowerCase();
      if(k==='w'||k==='arrowup'){ak='f';drive('f');} else if(k==='s'||k==='arrowdown'){ak='b';drive('b');}
      else if(k==='a'||k==='arrowleft'){ak='l';drive('l');} else if(k==='d'||k==='arrowright'){ak='r';drive('r');}
      else if(k===' '){drive('x');}
    });
    window.addEventListener('keyup', e => { if(ak){drive('x');ak=null;} });

    setInterval(()=>{
      fetch('/telemetry').then(r=>r.json()).then(d=>{
        document.getElementById('temp').innerText=d.temp.toFixed(1)+'°C';
        document.getElementById('hum').innerText=d.hum.toFixed(1)+'%';
        const se=document.getElementById('sonar');
        if(d.distance>0){se.innerText=d.distance.toFixed(1)+'cm';se.classList.toggle('alert',d.distance<15);}
        else{se.innerText='--';se.classList.remove('alert');}
        const ie=document.getElementById('ir');
        ie.innerText=d.ir===0?'OBST!':'OK'; ie.classList.toggle('alert',d.ir===0);
        document.getElementById('ldr').innerText=d.light===1?'☀':'🌙';
        document.getElementById('action').innerText=d.action;
        document.getElementById('pwm').innerText=d.pwm;
      }).catch(()=>{});
    },300);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", DASHBOARD_HTML);
}

void handleTelemetry() {
  String json = "{";
  json += "\"temp\":" + String(currentTemp, 1) + ",";
  json += "\"hum\":" + String(currentHum, 1) + ",";
  json += "\"light\":" + String(currentLight) + ",";
  json += "\"ir\":" + String(currentIR) + ",";
  json += "\"distance\":" + String(currentDistance, 1) + ",";
  json += "\"action\":\"" + currentAction + "\",";
  json += "\"pwm\":" + String(currentPWM) + ",";
  json += "\"speed\":" + String(manualSpeed) + ",";
  json += "\"mode\":\"" + String(currentMode == MODE_AUTONOMOUS ? "auto" : "manual") + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleDrive() {
  if (!server.hasArg("dir")) {
    server.send(400, "text/plain", "Missing dir");
    return;
  }
  String dir = server.arg("dir");
  int spd = manualSpeed;
  if (server.hasArg("spd")) {
    spd = server.arg("spd").toInt();
    manualSpeed = constrain(spd, MIN_PWM, MAX_PWM);
  }

  if (currentMode == MODE_MANUAL) {
    if (dir == "f") driveForward(manualSpeed);
    else if (dir == "b") driveBackward(manualSpeed);
    else if (dir == "l") spinLeft(manualSpeed);
    else if (dir == "r") spinRight(manualSpeed);
    else if (dir == "x") stopRover();
  }
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("v")) {
    manualSpeed = constrain(server.arg("v").toInt(), MIN_PWM, MAX_PWM);
  }
  server.send(200, "text/plain", String(manualSpeed));
}

void handleMode() {
  if (server.hasArg("m")) {
    String m = server.arg("m");
    if (m == "auto") {
      currentMode = MODE_AUTONOMOUS;
      autoState = AUTO_CRUISE;
      autoStateStartMs = millis();
    } else {
      currentMode = MODE_MANUAL;
      stopRover();
    }
  }
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n========================================================"));
  Serial.println(F("  MICRO-ROS ROVER - PWM SPEED CONTROL & SMART NAVIGATION"));
  Serial.println(F("========================================================"));

  // Initialize sensors
  dht.begin();
  pinMode(LDR_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Initialize motors with PWM
  setupMotorPWM();
  stopRover();

  Serial.printf("Motor PWM initialized: %d Hz, %d-bit resolution\n", PWM_FREQ, PWM_RES);
  Serial.printf("Default manual speed: %d / 255\n", manualSpeed);

  // Setup Access Point Hotspot
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("Hotspot: "));
  Serial.println(AP_SSID);
  Serial.print(F("Dashboard: http://"));
  Serial.println(WiFi.softAPIP());

  // Try Station Mode
  if (String(STA_SSID) != "YOUR_WIFI_NAME") {
    Serial.printf("Connecting to Wi-Fi: %s ...\n", STA_SSID);
    WiFi.begin(STA_SSID, STA_PASS);
    unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < 8000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("\nConnected to Home Wi-Fi!"));
      Serial.print(F("LAN Dashboard: http://"));
      Serial.println(WiFi.localIP());
    } else {
      Serial.println(F("\nWi-Fi not connected. AP mode active."));
    }
  }

  // Web routes
  server.on("/", handleRoot);
  server.on("/telemetry", handleTelemetry);
  server.on("/drive", handleDrive);
  server.on("/speed", handleSpeed);
  server.on("/mode", handleMode);
  server.begin();

  Serial.println(F("HTTP Web Server Started!"));
}

void loop() {
  server.handleClient();

  // Sample sensors every 250ms
  if (millis() - lastSensorReadMs >= 250) {
    lastSensorReadMs = millis();
    updateSensors();
    // Serial telemetry for Mac dashboard
    Serial.printf("[TELEM] Temp:%.1fC | Hum:%.1f%% | Sonar:%.1fcm | IR:%d | LDR:%d | PWM:%d | Spd:%d\n",
                  currentTemp, currentHum, currentDistance, currentIR, currentLight, currentPWM, manualSpeed);
  }

  // Drive logic
  if (currentMode == MODE_AUTONOMOUS) {
    handleAutonomousBehavior();
  } else {
    // Manual: auto-stop if no heartbeat
    if (currentAction != "STOPPED" && (millis() - lastActionTime > ACTION_TIMEOUT_MS)) {
      stopRover();
    }
  }
}
