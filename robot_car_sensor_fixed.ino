#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

// ================= WIFI CREDENTIALS =================
const char* ssid = "wifi";
const char* password = "1234567890";

// ================= MOTOR PINS (L293D) =================
#define IN1 5    // GPIO5  (D1)
#define IN2 4    // GPIO4  (D2)
#define IN3 0    // GPIO0  (D3)
#define IN4 2    // GPIO2  (D4)

// ================= ULTRASONIC SENSOR =================
#define TRIG 14  // GPIO14 (D5)
#define ECHO 12  // GPIO12 (D6)

long duration;
int distance;

// ================= OBSTACLE AVOIDANCE PARAMETERS =================
#define SAFE_DISTANCE 35        // Safe distance in cm (increased)
#define CRITICAL_DISTANCE 20    // Critical distance - immediate stop (increased)
#define TURN_TIME 350           // Time to turn in ms
#define BACKUP_TIME 400         // Time to backup when stuck
#define SCAN_DELAY 150          // Delay between scans
#define NUM_READINGS 3          // Number of sensor readings to average

// ================= MODE CONTROL =================
bool autopilotMode = true;
bool engineOn = false;
String currentCommand = "STOP";

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
            
            // Visual warning if too close
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

// ================= SETUP =================
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  Serial.begin(9600);
  Serial.println("\n=== ROBOT CAR STARTING ===");
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("=========================");
  
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
        stopMotor();
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
    request->send(200, "text/plain", String(distance));
  });
  
  server.begin();
}

// ================= LOOP =================
void loop() {
  // Get stable distance reading
  distance = getStableDistance();
  
  if (engineOn && autopilotMode) {
    smartAutopilot();
  } else if (engineOn && !autopilotMode) {
    // Manual control
    if (currentCommand == "FORWARD") {
      moveForward();
    } else if (currentCommand == "BACKWARD") {
      moveBackward();
    } else if (currentCommand == "LEFT") {
      turnLeft();
    } else if (currentCommand == "RIGHT") {
      turnRight();
    } else if (currentCommand == "STOP") {
      stopMotor();
    }
  } else {
    stopMotor();
  }
  
  delay(50);
}

// ================= ADVANCED AUTOPILOT FUNCTION =================
void smartAutopilot() {
  int frontDistance = distance;
  
  // Debug output
  Serial.print("Distance: ");
  Serial.print(frontDistance);
  Serial.println(" cm");
  
  // CRITICAL: Too close - EMERGENCY STOP
  if (frontDistance > 0 && frontDistance < CRITICAL_DISTANCE) {
    Serial.println("!!! EMERGENCY - TOO CLOSE !!!");
    stopMotor();
    delay(300);
    
    // Back up
    Serial.println("Backing up...");
    moveBackward();
    delay(BACKUP_TIME);
    stopMotor();
    delay(300);
    
    // Find best escape route
    findBestPath();
    return;
  }
  
  // WARNING: Obstacle detected
  if (frontDistance > 0 && frontDistance < SAFE_DISTANCE) {
    Serial.println("Warning: Obstacle ahead!");
    stopMotor();
    delay(300);
    
    // Scan for better path
    findBestPath();
    return;
  }
  
  // Path is clear - move forward
  Serial.println("Path clear - Moving");
  moveForward();
}

// ================= INTELLIGENT PATH FINDING =================
void findBestPath() {
  Serial.println("\n=== PATH SCAN START ===");
  
  stopMotor();
  delay(400);
  
  // Get current front reading
  int frontDist = getStableDistance();
  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.println(" cm");
  
  // Scan RIGHT
  Serial.println("Scanning RIGHT...");
  turnRight();
  delay(TURN_TIME);
  stopMotor();
  delay(SCAN_DELAY);
  int rightDist = getStableDistance();
  Serial.print("Right: ");
  Serial.print(rightDist);
  Serial.println(" cm");
  
  // Return to center
  turnLeft();
  delay(TURN_TIME);
  stopMotor();
  delay(SCAN_DELAY);
  
  // Scan LEFT
  Serial.println("Scanning LEFT...");
  turnLeft();
  delay(TURN_TIME);
  stopMotor();
  delay(SCAN_DELAY);
  int leftDist = getStableDistance();
  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.println(" cm");
  
  // Return to center
  turnRight();
  delay(TURN_TIME);
  stopMotor();
  delay(SCAN_DELAY);
  
  // Make decision
  Serial.println("=== DECISION ===");
  
  // If all paths blocked - back up and turn around
  if (leftDist < SAFE_DISTANCE && rightDist < SAFE_DISTANCE && frontDist < SAFE_DISTANCE) {
    Serial.println("All paths blocked! Emergency backup!");
    moveBackward();
    delay(BACKUP_TIME * 2);
    stopMotor();
    delay(300);
    
    // Turn 180 degrees
    Serial.println("Turning around...");
    turnRight();
    delay(TURN_TIME * 2.5);
    stopMotor();
    delay(300);
    return;
  }
  
  // Choose best path
  if (leftDist > rightDist && leftDist > frontDist) {
    Serial.println("Decision: Turn LEFT");
    turnLeft();
    delay(TURN_TIME * 1.8);
  } else if (rightDist > leftDist && rightDist > frontDist) {
    Serial.println("Decision: Turn RIGHT");
    turnRight();
    delay(TURN_TIME * 1.8);
  } else {
    Serial.println("Decision: Continue forward");
  }
  
  stopMotor();
  delay(300);
  Serial.println("=== SCAN COMPLETE ===\n");
}

// ================= STABLE DISTANCE READING WITH AVERAGING =================
int getStableDistance() {
  int readings[NUM_READINGS];
  int validCount = 0;
  
  // Take multiple readings
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = getSingleDistance();
    
    // Only count valid readings (not 0 or too high)
    if (readings[i] > 0 && readings[i] < 400) {
      validCount++;
    }
    
    delay(10);  // Small delay between readings
  }
  
  // If no valid readings, assume path is clear
  if (validCount == 0) {
    return 200;  // Return large value for clear path
  }
  
  // Calculate median (more stable than average for noisy data)
  // Sort the array
  for (int i = 0; i < NUM_READINGS - 1; i++) {
    for (int j = i + 1; j < NUM_READINGS; j++) {
      if (readings[i] > readings[j]) {
        int temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  
  // Return median value
  int median = readings[NUM_READINGS / 2];
  
  // Final validation
  if (median < 2) {
    // Too close or error - treat as obstacle
    return 5;  // Very close obstacle
  }
  
  if (median > 400) {
    // Too far or no echo - path is clear
    return 200;
  }
  
  return median;
}

// ================= SINGLE DISTANCE READING =================
int getSingleDistance() {
  // Ensure trigger is low
  digitalWrite(TRIG, LOW);
  delayMicroseconds(5);
  
  // Send 10us pulse
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  // Wait for echo with timeout
  long duration = pulseIn(ECHO, HIGH, 25000);  // 25ms timeout
  
  // If timeout (no echo received)
  if (duration == 0) {
    return 400;  // Assume far away
  }
  
  // Calculate distance in cm
  int dist = duration * 0.034 / 2;
  
  return dist;
}

// ================= MOTOR FUNCTIONS =================
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
