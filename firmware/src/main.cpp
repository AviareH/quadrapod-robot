#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>

// --- WIFI CREDENTIALS ---
const char* ssid     = "Horizons";
const char* password = "ithurtswhenip";

// --- PWM DRIVER ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- PULSE WIDTH CALIBRATION ---
// PCA9685 uses 12-bit resolution (0-4096 ticks) at 50Hz (20ms period)
// 1 tick = 20ms / 4096 = ~4.88 microseconds
// Standard servo pulse: 1ms (0deg) to 2ms (180deg)
// 1ms = ~205 ticks, 2ms = ~410 ticks
// Start conservative - widen carefully during calibration
#define SERVOMIN 130
#define SERVOMAX 520

// --- SERVO TABLE ---
const int NUM_SERVOS = 12;

struct ServoInfo {
  uint8_t channel;
  const char* name;
  const char* leg;
  int currentAngle;
};

ServoInfo servos[NUM_SERVOS] = {
  {0,  "Foot", "FL", 90},
  {1,  "Knee", "FL", 90},
  {2,  "Hip",  "FL", 90},
  {4,  "Foot", "FR", 90},
  {5,  "Knee", "FR", 90},
  {6,  "Hip",  "FR", 90},
  {8,  "Foot", "BR", 90},
  {9,  "Knee", "BR", 90},
  {10, "Hip",  "BR", 90},
  {12, "Foot", "BL", 90},
  {13, "Knee", "BL", 90},
  {14, "Hip",  "BL", 90}
};

WebServer server(80);

// --- SERVO CONTROL ---
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

// --- HTML PAGE ---
// Served once on page load. All subsequent control is via fetch() API calls.
const char* HTML = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Quadruped Servo Calibration</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg:       #0d0f12;
    --surface:  #151820;
    --border:   #252a35;
    --accent:   #3d7fff;
    --accent2:  #00c896;
    --text:     #e2e6f0;
    --muted:    #6b7385;
    --warn:     #ff6b35;
    --radius:   10px;
  }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'SF Mono', 'Fira Code', 'Consolas', monospace;
    min-height: 100vh;
    padding: 24px 16px;
  }

  header {
    display: flex;
    align-items: baseline;
    gap: 16px;
    margin-bottom: 28px;
    padding-bottom: 16px;
    border-bottom: 1px solid var(--border);
  }

  header h1 {
    font-size: 18px;
    font-weight: 600;
    letter-spacing: 0.04em;
    color: var(--text);
  }

  header span {
    font-size: 12px;
    color: var(--muted);
  }

  #status-bar {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 24px;
    font-size: 12px;
    color: var(--muted);
  }

  #status-dot {
    width: 7px;
    height: 7px;
    border-radius: 50%;
    background: var(--accent2);
    box-shadow: 0 0 6px var(--accent2);
    flex-shrink: 0;
  }

  .warning-box {
    background: rgba(255,107,53,0.08);
    border: 1px solid rgba(255,107,53,0.3);
    border-radius: var(--radius);
    padding: 12px 16px;
    font-size: 12px;
    color: #ffaa80;
    margin-bottom: 24px;
    line-height: 1.6;
  }

  .warning-box strong { color: var(--warn); }

  /* ALL SERVOS button */
  .global-row {
    display: flex;
    gap: 10px;
    margin-bottom: 24px;
    flex-wrap: wrap;
  }

  .global-btn {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    color: var(--text);
    font-family: inherit;
    font-size: 12px;
    padding: 8px 16px;
    cursor: pointer;
    transition: border-color 0.15s, background 0.15s;
  }

  .global-btn:hover { border-color: var(--accent); background: rgba(61,127,255,0.06); }

  /* Leg groups */
  .legs-grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 14px;
  }

  @media (max-width: 600px) {
    .legs-grid { grid-template-columns: 1fr; }
  }

  .leg-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    overflow: hidden;
  }

  .leg-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 14px;
    border-bottom: 1px solid var(--border);
    background: rgba(255,255,255,0.02);
  }

  .leg-label {
    font-size: 13px;
    font-weight: 600;
    letter-spacing: 0.06em;
    color: var(--accent);
  }

  .leg-neutral-btn {
    font-family: inherit;
    font-size: 11px;
    background: rgba(61,127,255,0.1);
    border: 1px solid rgba(61,127,255,0.25);
    color: var(--accent);
    border-radius: 6px;
    padding: 4px 10px;
    cursor: pointer;
    transition: background 0.15s;
  }

  .leg-neutral-btn:hover { background: rgba(61,127,255,0.2); }

  /* Servo rows */
  .servo-row {
    padding: 12px 14px;
    border-bottom: 1px solid var(--border);
    display: grid;
    grid-template-columns: 52px 1fr 56px 80px;
    align-items: center;
    gap: 10px;
  }

  .servo-row:last-child { border-bottom: none; }

  .servo-joint {
    font-size: 11px;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }

  input[type="range"] {
    -webkit-appearance: none;
    width: 100%;
    height: 4px;
    border-radius: 2px;
    background: var(--border);
    outline: none;
    cursor: pointer;
  }

  input[type="range"]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 14px;
    height: 14px;
    border-radius: 50%;
    background: var(--accent);
    cursor: pointer;
    box-shadow: 0 0 6px rgba(61,127,255,0.4);
    transition: box-shadow 0.15s;
  }

  input[type="range"]::-webkit-slider-thumb:hover {
    box-shadow: 0 0 10px rgba(61,127,255,0.7);
  }

  .angle-display {
    font-size: 13px;
    font-weight: 600;
    color: var(--accent2);
    text-align: right;
    min-width: 40px;
  }

  .angle-input {
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--text);
    font-family: inherit;
    font-size: 12px;
    padding: 4px 8px;
    width: 100%;
    text-align: center;
    -moz-appearance: textfield;
  }

  .angle-input::-webkit-outer-spin-button,
  .angle-input::-webkit-inner-spin-button { -webkit-appearance: none; }

  .angle-input:focus {
    outline: none;
    border-color: var(--accent);
  }

  /* Toast */
  #toast {
    position: fixed;
    bottom: 24px;
    right: 24px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 10px 16px;
    font-size: 12px;
    color: var(--accent2);
    opacity: 0;
    transform: translateY(8px);
    transition: opacity 0.2s, transform 0.2s;
    pointer-events: none;
    z-index: 999;
  }

  #toast.show {
    opacity: 1;
    transform: translateY(0);
  }
</style>
</head>
<body>

<header>
  <h1>SERVO CALIBRATION</h1>
  <span>quadruped · 12 servos · PCA9685</span>
</header>

<div id="status-bar">
  <div id="status-dot"></div>
  <span id="status-text">Connected to ESP32</span>
</div>

<div class="warning-box">
  <strong>⚠ Grinding = hard stop.</strong> If you hear the servo straining, back off immediately.
  SERVOMIN/SERVOMAX are conservative — widen in firmware once physical limits are confirmed.
</div>

<div class="global-row">
  <button class="global-btn" onclick="allTo(90)">All → 90°</button>
  <button class="global-btn" onclick="allTo(0)">All → 0°</button>
  <button class="global-btn" onclick="allTo(180)">All → 180°</button>
</div>

<div class="legs-grid" id="legs-grid"></div>

<div id="toast"></div>

<script>
const SERVOS = [
  {idx:0,  ch:0,  leg:"FL", joint:"Foot"},
  {idx:1,  ch:1,  leg:"FL", joint:"Knee"},
  {idx:2,  ch:2,  leg:"FL", joint:"Hip"},
  {idx:3,  ch:4,  leg:"FR", joint:"Foot"},
  {idx:4,  ch:5,  leg:"FR", joint:"Knee"},
  {idx:5,  ch:6,  leg:"FR", joint:"Hip"},
  {idx:6,  ch:8,  leg:"BR", joint:"Foot"},
  {idx:7,  ch:9,  leg:"BR", joint:"Knee"},
  {idx:8,  ch:10, leg:"BR", joint:"Hip"},
  {idx:9,  ch:12, leg:"BL", joint:"Foot"},
  {idx:10, ch:13, leg:"BL", joint:"Knee"},
  {idx:11, ch:14, leg:"BL", joint:"Hip"},
];

const LEGS = ["FL","FR","BR","BL"];
const angles = {};
SERVOS.forEach(s => angles[s.idx] = 90);

// Build UI
const grid = document.getElementById('legs-grid');
LEGS.forEach(leg => {
  const legServos = SERVOS.filter(s => s.leg === leg);
  const card = document.createElement('div');
  card.className = 'leg-card';
  card.innerHTML = `
    <div class="leg-header">
      <span class="leg-label">${leg}</span>
      <button class="leg-neutral-btn" onclick="legTo('${leg}', 90)">Neutral</button>
    </div>
    ${legServos.map(s => `
    <div class="servo-row">
      <span class="servo-joint">${s.joint}</span>
      <input type="range" min="0" max="180" value="90"
        id="slider-${s.idx}"
        oninput="onSlider(${s.idx}, this.value)">
      <span class="angle-display" id="disp-${s.idx}">90°</span>
      <input type="number" class="angle-input" min="0" max="180" value="90"
        id="input-${s.idx}"
        onchange="onInput(${s.idx}, this.value)"
        onkeydown="if(event.key==='Enter') onInput(${s.idx}, this.value)">
    </div>`).join('')}
  `;
  grid.appendChild(card);
});

function updateUI(idx, angle) {
  angle = Math.max(0, Math.min(180, parseInt(angle)));
  angles[idx] = angle;
  document.getElementById(`slider-${idx}`).value = angle;
  document.getElementById(`disp-${idx}`).textContent = angle + '°';
  document.getElementById(`input-${idx}`).value = angle;
}

function sendAngle(idx, angle) {
  angle = Math.max(0, Math.min(180, parseInt(angle)));
  updateUI(idx, angle);
  fetch(`/set?idx=${idx}&angle=${angle}`)
    .then(r => r.text())
    .then(() => showToast(`${SERVOS[idx].leg} ${SERVOS[idx].joint} → ${angle}°`))
    .catch(() => {
      document.getElementById('status-dot').style.background = '#ff4444';
      document.getElementById('status-text').textContent = 'Connection lost';
    });
}

function onSlider(idx, val) { sendAngle(idx, val); }
function onInput(idx, val)  { sendAngle(idx, val); }

function allTo(angle) {
  SERVOS.forEach(s => sendAngle(s.idx, angle));
}

function legTo(leg, angle) {
  SERVOS.filter(s => s.leg === leg).forEach(s => sendAngle(s.idx, angle));
}

let toastTimer;
function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 1500);
}
</script>
</body>
</html>
)rawhtml";

// --- HTTP HANDLERS ---
void handleRoot() {
  server.send(200, "text/html", HTML);
}

void handleSet() {
  if (!server.hasArg("idx") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "Missing args");
    return;
  }

  int idx   = server.arg("idx").toInt();
  int angle = server.arg("angle").toInt();

  if (idx < 0 || idx >= NUM_SERVOS) {
    server.send(400, "text/plain", "Invalid servo index");
    return;
  }

  angle = constrain(angle, 0, 180);
  servos[idx].currentAngle = angle;
  setServoAngle(servos[idx].channel, angle);

  Serial.printf("[SET] %s %s (ch%d) -> %d deg\n",
    servos[idx].leg, servos[idx].name,
    servos[idx].channel, angle);

  server.send(200, "text/plain", "OK");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Centre all servos on boot
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(servos[i].channel, 90);
  }

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. Open: http://");
  Serial.println(WiFi.localIP());

  // Routes
  server.on("/",    handleRoot);
  server.on("/set", handleSet);
  server.onNotFound(handleNotFound);
  server.begin();
}

// --- LOOP ---
void loop() {
  server.handleClient();
}