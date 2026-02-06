/******************************************************
 * ESP8266 NodeMCU Advanced Obstacle-Avoiding Robot
 * Web Interface + Improved Autopilot
 * L293D Motor Driver + HC-SR04 Ultrasonic Sensor
 ******************************************************/

#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

// ================= WIFI CREDENTIALS =================
const char* ssid = "wifi";
const char* password = "1234567890";

// ================= MOTOR PINS (L293D) =================
// Left Motor
const int ENA = D3;       // GPIO0 - Left motor speed control
const int IN1 = D1;       // GPIO5 - Left motor direction 1
const int IN2 = D2;       // GPIO4 - Left motor direction 2

// Right Motor
const int ENB = D4;       // GPIO2 - Right motor speed control
const int IN3 = D7;       // GPIO13 - Right motor direction 1
const int IN4 = D8;       // GPIO15 - Right motor direction 2

// ================= ULTRASONIC SENSOR =================
const int TRIG = D6;      // GPIO12
const int ECHO = D5;      // GPIO14

// ================= SPEED SETTINGS =================
const int SPEED_FORWARD  = 780;
const int SPEED_TURN     = 700;
const int SPEED_BACKWARD = 650;

// ================= DISTANCE THRESHOLDS =================
const int DIST_MIN_CM = 22;        // Minimum safe distance
const int CRITICAL_DISTANCE = 15;   // Emergency stop distance

// ================= TIMING PARAMETERS =================
const int TURN_90_TIME = 520;       // Time for 90° turn
const int SMALL_TURN_TIME = 150;    // Time for small scan turn
const int BACKUP_TIME = 200;        // Backup duration

// ================= ULTRASONIC TIMEOUT =================
unsigned long echoTimeoutUs = 25000UL;

// ================= MODE CONTROL =================
bool autopilotMode = true;
bool engineOn = false;
String currentCommand = "STOP";
float distance = 0;

// ================= WEB SERVER =================
AsyncWebServer server(80);

// ================= HTML WEB PAGE =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Car Control</title>
  <style>
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }
    
    body {
      font-family: 'Arial', sans-serif;
      text-align: center;
      background: linear-gradient(135deg, #1e3c72 0%, #2a5298 50%, #7e22ce 100%);
      color: white;
      min-height: 100vh;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow-x: hidden;
      padding: 20px;
    }
    
    body::before {
      content: '';
      position: fixed;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      background: 
        radial-gradient(circle at 20% 50%, rgba(120, 119, 198, 0.3), transparent 50%),
        radial-gradient(circle at 80% 80%, rgba(138, 43, 226, 0.3), transparent 50%);
      animation: pulse 15s ease-in-out infinite;
      z-index: -1;
    }
    
    @keyframes pulse {
      0%, 100% { opacity: 0.5; }
      50% { opacity: 0.8; }
    }
    
    .container {
      max-width: 600px;
      width: 100%;
      background: rgba(255, 255, 255, 0.1);
      backdrop-filter: blur(20px);
      -webkit-backdrop-filter: blur(20px);
      padding: 30px 20px;
      border-radius: 25px;
      box-shadow: 
        0 8px 32px rgba(0, 0, 0, 0.3),
        inset 0 1px 0 rgba(255, 255, 255, 0.2);
      border: 1px solid rgba(255, 255, 255, 0.2);
      animation: slideIn 0.6s ease-out;
    }
    
    @keyframes slideIn {
      from {
        opacity: 0;
        transform: translateY(30px);
      }
      to {
        opacity: 1;
        transform: translateY(0);
      }
    }
    
    h1 {
      margin-bottom: 5px;
      font-size: 2em;
      font-weight: 700;
      color: white;
      text-shadow: 0 0 20px rgba(255, 255, 255, 0.5);
    }
    
    .subtitle {
      color: rgba(255, 255, 255, 0.8);
      margin-bottom: 25px;
      font-size: 0.85em;
      letter-spacing: 2px;
      text-transform: uppercase;
    }
    
    .ignition-modal {
      position: fixed;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      background: rgba(0, 0, 0, 0.95);
      display: flex;
      align-items: center;
      justify-content: center;
      z-index: 1000;
      animation: fadeIn 0.3s;
    }
    
    .ignition-modal.hidden {
      display: none;
    }
    
    @keyframes fadeIn {
      from { opacity: 0; }
      to { opacity: 1; }
    }
    
    .key-container {
      text-align: center;
      animation: bounceIn 0.6s;
    }
    
    @keyframes bounceIn {
      0% { transform: scale(0.3); opacity: 0; }
      50% { transform: scale(1.05); }
      100% { transform: scale(1); opacity: 1; }
    }
    
    .key-title {
      font-size: 1.5em;
      margin-bottom: 30px;
      color: white;
      text-shadow: 0 0 20px rgba(255, 255, 255, 0.5);
    }
    
    .key-button {
      width: 120px;
      height: 120px;
      border-radius: 50%;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      border: 4px solid rgba(255, 255, 255, 0.3);
      color: white;
      font-size: 2.5em;
      cursor: pointer;
      transition: all 0.3s;
      box-shadow: 
        0 10px 40px rgba(102, 126, 234, 0.6),
        inset 0 -5px 20px rgba(0, 0, 0, 0.3);
      display: flex;
      align-items: center;
      justify-content: center;
      margin: 0 auto;
    }
    
    .key-button:hover {
      transform: scale(1.1) rotate(90deg);
      box-shadow: 
        0 15px 60px rgba(102, 126, 234, 0.8),
        inset 0 -5px 20px rgba(0, 0, 0, 0.3);
    }
    
    .key-button:active {
      transform: scale(1.05) rotate(180deg);
    }
    
    .status-dashboard {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin: 20px 0;
    }
    
    .status-card {
      background: rgba(255, 255, 255, 0.15);
      padding: 15px;
      border-radius: 12px;
      border: 1px solid rgba(255, 255, 255, 0.2);
      transition: all 0.3s;
    }
    
    .status-card:hover {
      background: rgba(255, 255, 255, 0.2);
      transform: translateY(-3px);
    }
    
    .status-label {
      font-size: 0.75em;
      color: rgba(255, 255, 255, 0.7);
      margin-bottom: 5px;
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    
    .status-value {
      font-size: 1.5em;
      font-weight: bold;
      color: #FFD700;
      text-shadow: 0 0 10px rgba(255, 215, 0, 0.5);
    }
    
    .distance-warning {
      color: #ff4444 !important;
      animation: blink 1s infinite;
    }
    
    @keyframes blink {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }
    
    .engine-status {
      font-size: 1.1em;
      color: #4ade80;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
    }
    
    .engine-status.off {
      color: #ef4444;
    }
    
    .pulse-dot {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: #4ade80;
      animation: pulseDot 1.5s infinite;
    }
    
    .pulse-dot.off {
      background: #ef4444;
      animation: none;
    }
    
    @keyframes pulseDot {
      0%, 100% { opacity: 1; transform: scale(1); }
      50% { opacity: 0.5; transform: scale(1.2); }
    }
    
    .mode-toggle {
      margin: 20px 0;
    }
    
    .toggle-btn {
      background: linear-gradient(135deg, #10b981, #059669);
      color: white;
      border: none;
      padding: 15px 35px;
      font-size: 1.1em;
      font-weight: 600;
      border-radius: 12px;
      cursor: pointer;
      transition: all 0.3s;
      width: 100%;
      max-width: 280px;
      box-shadow: 0 6px 20px rgba(16, 185, 129, 0.4);
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    
    .toggle-btn:hover {
      transform: translateY(-2px);
      box-shadow: 0 10px 25px rgba(16, 185, 129, 0.6);
    }
    
    .toggle-btn.manual {
      background: linear-gradient(135deg, #f59e0b, #d97706);
      box-shadow: 0 6px 20px rgba(245, 158, 11, 0.4);
    }
    
    .toggle-btn.manual:hover {
      box-shadow: 0 10px 25px rgba(245, 158, 11, 0.6);
    }
    
    .toggle-btn:active {
      transform: translateY(0);
    }
    
    .controls {
      display: none;
      margin-top: 25px;
      animation: fadeInUp 0.4s;
    }
    
    .controls.active {
      display: block;
    }
    
    @keyframes fadeInUp {
      from {
        opacity: 0;
        transform: translateY(20px);
      }
      to {
        opacity: 1;
        transform: translateY(0);
      }
    }
    
    .controls h2 {
      margin-bottom: 15px;
      font-size: 1.3em;
      color: #fbbf24;
    }
    
    .control-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
      max-width: 320px;
      margin: 0 auto;
    }
    
    .control-btn {
      background: rgba(255, 255, 255, 0.2);
      border: 2px solid rgba(255, 255, 255, 0.4);
      color: white;
      padding: 20px;
      font-size: 1.8em;
      border-radius: 12px;
      cursor: pointer;
      transition: all 0.2s;
      box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2);
      user-select: none;
      -webkit-user-select: none;
      -webkit-tap-highlight-color: transparent;
    }
    
    .control-btn:hover {
      background: rgba(255, 255, 255, 0.3);
      transform: scale(1.05);
    }
    
    .control-btn:active {
      background: rgba(255, 255, 255, 0.4);
      transform: scale(0.95);
      box-shadow: 0 2px 8px rgba(0, 0, 0, 0.3);
    }
    
    .control-btn.stop {
      background: rgba(239, 68, 68, 0.4);
      border-color: #ef4444;
    }
    
    .control-btn.stop:active {
      background: rgba(239, 68, 68, 0.6);
    }
    
    @media (max-width: 480px) {
      h1 { font-size: 1.8em; }
      .control-btn { padding: 18px; font-size: 1.6em; }
      .status-value { font-size: 1.3em; }
      .container { padding: 25px 15px; }
    }
  </style>
</head>
<body>
  <div class="ignition-modal" id="ignitionModal">
    <div class="key-container">
      <div class="key-title">Turn the Key to Start</div>
      <button class="key-button" onclick="startEngine()">🔑</button>
    </div>
  </div>

  <div class="container">
    <h1>ROBO CONTROL</h1>
    <div class="subtitle">Advanced Navigation</div>
    
    <div class="status-dashboard">
      <div class="status-card">
        <div class="status-label">Distance</div>
        <div class="status-value" id="distanceDisplay"><span id="distance">--</span> cm</div>
      </div>
      <div class="status-card">
        <div class="status-label">Engine</div>
        <div class="engine-status off" id="engineStatus">
          <span class="pulse-dot off" id="pulseDot"></span>
          <span id="engineText">OFF</span>
        </div>
      </div>
    </div>
    
    <div class="status-card" style="margin: 12px 0;">
      <div class="status-label">Mode</div>
      <div class="status-value" style="font-size: 1.2em;" id="modeStatus">AUTOPILOT</div>
    </div>
    
    <div class="mode-toggle">
      <button class="toggle-btn" id="modeBtn" onclick="toggleMode()">
        Autopilot Mode
      </button>
    </div>
    
    <div class="controls" id="manualControls">
      <h2>Manual Control</h2>
      <div class="control-grid">
        <div></div>
        <button class="control-btn" 
          onmousedown="sendCommand('FORWARD')" 
          onmouseup="sendCommand('STOP')" 
          ontouchstart="sendCommand('FORWARD')" 
          ontouchend="sendCommand('STOP')">
          ▲
        </button>
        <div></div>
        
        <button class="control-btn" 
          onmousedown="sendCommand('LEFT')" 
          onmouseup="sendCommand('STOP')" 
          ontouchstart="sendCommand('LEFT')" 
          ontouchend="sendCommand('STOP')">
          ◄
        </button>
        <button class="control-btn stop" onclick="sendCommand('STOP')">■</button>
        <button class="control-btn" 
          onmousedown="sendCommand('RIGHT')" 
          onmouseup="sendCommand('STOP')" 
          ontouchstart="sendCommand('RIGHT')" 
          ontouchend="sendCommand('STOP')">
          ►
        </button>
        
        <div></div>
        <button class="control-btn" 
          onmousedown="sendCommand('BACKWARD')" 
          onmouseup="sendCommand('STOP')" 
          ontouchstart="sendCommand('BACKWARD')" 
          ontouchend="sendCommand('STOP')">
          ▼
        </button>
        <div></div>
      </div>
    </div>
  </div>

  <script>
    let autopilot = true;
    let engineOn = false;
    
    function startEngine() {
      fetch('/engine?state=on')
        .then(() => {
          engineOn = true;
          document.getElementById('ignitionModal').classList.add('hidden');
          document.getElementById('engineStatus').classList.remove('off');
          document.getElementById('pulseDot').classList.remove('off');
          document.getElementById('engineText').textContent = 'ON';
        });
    }
    
    function toggleMode() {
      if (!engineOn) {
        alert('Please turn on the engine first!');
        return;
      }
      
      autopilot = !autopilot;
      const btn = document.getElementById('modeBtn');
      const controls = document.getElementById('manualControls');
      const modeStatus = document.getElementById('modeStatus');
      
      if (autopilot) {
        btn.textContent = 'Autopilot Mode';
        btn.classList.remove('manual');
        controls.classList.remove('active');
        modeStatus.textContent = 'AUTOPILOT';
        modeStatus.style.color = '#4ade80';
        fetch('/mode?value=auto');
      } else {
        btn.textContent = 'Manual Mode';
        btn.classList.add('manual');
        controls.classList.add('active');
        modeStatus.textContent = 'MANUAL';
        modeStatus.style.color = '#fbbf24';
        fetch('/mode?value=manual');
      }
    }
    
    function sendCommand(cmd) {
      if (!engineOn) return;
      if (!autopilot) {
        fetch('/control?cmd=' + cmd);
      }
    }
    
    setInterval(function() {
      if (engineOn) {
        fetch('/distance')
          .then(response => response.text())
          .then(data => {
            const distValue = parseInt(data);
            document.getElementById('distance').textContent = data;
            
            const distDisplay = document.getElementById('distanceDisplay');
            if (distValue < 20 && distValue > 0) {
              distDisplay.classList.add('distance-warning');
            } else {
              distDisplay.classList.remove('distance-warning');
            }
          });
      }
    }, 500);
    
    document.addEventListener('touchstart', function(e) {
      if (e.target.classList.contains('control-btn')) {
        e.preventDefault();
      }
    }, { passive: false });
  </script>
</body>
</html>
)rawliteral";

// ================= DISTANCE READING (MEDIAN OF 3) =================
float readDistanceCm() {
  float readings[3];

  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    unsigned long duration = pulseIn(ECHO, HIGH, echoTimeoutUs);
    readings[i] = (duration == 0) ? 9999.0 : (duration * 0.0343 * 0.5);
    delay(5);
  }

  // Sort array to find median (bubble sort)
  if (readings[0] > readings[1]) { 
    float temp = readings[0]; 
    readings[0] = readings[1]; 
    readings[1] = temp; 
  }
  if (readings[1] > readings[2]) { 
    float temp = readings[1]; 
    readings[1] = readings[2]; 
    readings[2] = temp; 
  }
  if (readings[0] > readings[1]) { 
    float temp = readings[0]; 
    readings[0] = readings[1]; 
    readings[1] = temp; 
  }

  return readings[1]; // Return median value
}

// ================= MOTOR CONTROL FUNCTIONS =================
void motorsStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void motorsForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void motorsBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void motorsTurnLeft(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void motorsTurnRight(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

// ================= SCANNING FUNCTIONS =================
float scanLeftSmall() {
  motorsTurnLeft(650);
  delay(SMALL_TURN_TIME);
  motorsStop();
  delay(60);
  return readDistanceCm();
}

float scanRightSmall() {
  motorsTurnRight(650);
  delay(SMALL_TURN_TIME);
  motorsStop();
  delay(60);
  return readDistanceCm();
}

void turnLeft90() {
  motorsTurnLeft(SPEED_TURN);
  delay(TURN_90_TIME);
  motorsStop();
  delay(80);
}

void turnRight90() {
  motorsTurnRight(SPEED_TURN);
  delay(TURN_90_TIME);
  motorsStop();
  delay(80);
}

// ================= ADVANCED AUTOPILOT FUNCTION =================
void smartAutopilot() {
  float frontDistance = readDistanceCm();
  distance = frontDistance; // Update global distance for web interface
  
  Serial.print("Front Distance: ");
  Serial.print(frontDistance);
  Serial.println(" cm");

  // Normal forward movement - path is clear
  if (frontDistance > DIST_MIN_CM) {
    Serial.println("Path clear - Moving forward");
    motorsForward(SPEED_FORWARD);
    return;
  }

  // Obstacle detected - stop and analyze
  Serial.println("Obstacle detected!");
  motorsStop();
  delay(80);

  // Step 1: Small scans left and right
  Serial.println("Scanning left...");
  float leftDist1 = scanLeftSmall();
  Serial.print("Left: ");
  Serial.print(leftDist1);
  Serial.println(" cm");
  
  Serial.println("Scanning right...");
  float rightDist1 = scanRightSmall();
  Serial.print("Right: ");
  Serial.print(rightDist1);
  Serial.println(" cm");

  // If left is clear, turn left
  if (leftDist1 > rightDist1 && leftDist1 > DIST_MIN_CM) {
    Serial.println("Decision: Turn LEFT");
    motorsTurnLeft(SPEED_TURN);
    delay(300);
    motorsStop();
    return;
  }

  // If right is clear, turn right
  if (rightDist1 > leftDist1 && rightDist1 > DIST_MIN_CM) {
    Serial.println("Decision: Turn RIGHT");
    motorsTurnRight(SPEED_TURN);
    delay(300);
    motorsStop();
    return;
  }

  // Step 2: Both sides blocked - backup and try again
  Serial.println("Both sides blocked - Backing up");
  motorsBackward(SPEED_BACKWARD);
  delay(BACKUP_TIME);
  motorsStop();
  delay(80);

  // Step 3: Scan again after backing up
  Serial.println("Re-scanning after backup...");
  float leftDist2 = scanLeftSmall();
  float rightDist2 = scanRightSmall();

  if (leftDist2 > rightDist2 && leftDist2 > DIST_MIN_CM) {
    Serial.println("Decision: Turn LEFT");
    motorsTurnLeft(SPEED_TURN);
    delay(320);
    motorsStop();
    return;
  }

  if (rightDist2 > leftDist2 && rightDist2 > DIST_MIN_CM) {
    Serial.println("Decision: Turn RIGHT");
    motorsTurnRight(SPEED_TURN);
    delay(320);
    motorsStop();
    return;
  }

  // Step 4: DEAD END - Hard escape maneuver
  Serial.println("!!! DEAD END DETECTED !!!");
  Serial.println("Executing 90° escape turn");
  
  turnLeft90();
  delay(80);

  // Check if path is now clear
  if (readDistanceCm() > DIST_MIN_CM) {
    Serial.println("Escape successful!");
    return;
  }

  // Still blocked - perform 180° turn
  Serial.println("Still blocked - 180° turn");
  turnRight90();
  turnRight90();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ROBOT CAR STARTING ===");

  // Motor pins setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  motorsStop();

  // WiFi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("=========================");

  // Web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/engine", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("state")) {
      String state = request->getParam("state")->value();
      engineOn = (state == "on");
      Serial.println(engineOn ? "ENGINE ON" : "ENGINE OFF");
      request->send(200, "text/plain", "OK");
    }
  });

  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      String mode = request->getParam("value")->value();
      autopilotMode = (mode == "auto");
      Serial.println(autopilotMode ? "MODE: AUTOPILOT" : "MODE: MANUAL");
      if (!autopilotMode) {
        motorsStop();
      }
      request->send(200, "text/plain", "OK");
    }
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("cmd")) {
      currentCommand = request->getParam("cmd")->value();
      Serial.print("Manual Command: ");
      Serial.println(currentCommand);
      request->send(200, "text/plain", "OK");
    }
  });

  server.on("/distance", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String((int)distance));
  });

  server.begin();
  Serial.println("Web server started!");
}

// ================= MAIN LOOP =================
void loop() {
  if (engineOn && autopilotMode) {
    // AUTOPILOT MODE - Use advanced obstacle avoidance
    smartAutopilot();
  } 
  else if (engineOn && !autopilotMode) {
    // MANUAL CONTROL MODE
    if (currentCommand == "FORWARD") {
      motorsForward(SPEED_FORWARD);
    } 
    else if (currentCommand == "BACKWARD") {
      motorsBackward(SPEED_BACKWARD);
    } 
    else if (currentCommand == "LEFT") {
      motorsTurnLeft(SPEED_TURN);
    } 
    else if (currentCommand == "RIGHT") {
      motorsTurnRight(SPEED_TURN);
    } 
    else if (currentCommand == "STOP") {
      motorsStop();
    }
    
    // Update distance in manual mode too
    distance = readDistanceCm();
  } 
  else {
    // Engine OFF - stop all motors
    motorsStop();
  }

  delay(50);
}
