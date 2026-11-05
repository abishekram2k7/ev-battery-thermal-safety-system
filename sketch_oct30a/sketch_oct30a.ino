#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

const char* ap_ssid = "EV_Battery_Node";
const char* ap_pass = "safety123";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// HardwareSerial2: RX on GPIO 16 (from STM32 PA9), TX on GPIO 17 (to STM32 PA10)
HardwareSerial STM_Serial(2);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>EV Battery Protection Supervisor</title>
  <style>
    :root {
      --bg: #0b1120;
      --card: #1e293b;
      --safe: #10b981;
      --warn: #f59e0b;
      --danger: #ef4444;
      --text: #f8fafc;
      --sub: #94a3b8;
    }
    body {
      margin: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: var(--bg);
      color: var(--text);
      padding: 16px;
    }
    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding-bottom: 12px;
      border-bottom: 1px solid #334155;
      margin-bottom: 16px;
    }
    .badge {
      font-weight: 700;
      padding: 6px 14px;
      border-radius: 9999px;
      font-size: 0.85rem;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 12px;
      margin-bottom: 16px;
    }
    .card {
      background: var(--card);
      border-radius: 8px;
      padding: 18px;
      text-align: center;
      border: 1px solid #334155;
    }
    .card-label { font-size: 0.75rem; color: var(--sub); text-transform: uppercase; }
    .card-val { font-size: 1.8rem; font-weight: 800; margin-top: 8px; }
    .controls {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin-top: 16px;
    }
    .btn {
      padding: 16px;
      border: none;
      border-radius: 8px;
      font-weight: 700;
      font-size: 0.95rem;
      cursor: pointer;
      color: #fff;
    }
    .btn-trip { background: var(--danger); }
    .btn-on   { background: var(--safe); }
  </style>
</head>
<body>
  <div class="header">
    <div>
      <h3 style="margin:0;">EV Safety Supervisor</h3>
      <span style="font-size:0.75rem; color:var(--sub);">Live Link: STM32F446RE</span>
    </div>
    <div id="statusBadge" class="badge" style="background:var(--safe);">NORMAL</div>
  </div>

  <div class="grid">
    <div class="card">
      <div class="card-label">DS18B20 Temp</div>
      <div id="t" class="card-val" style="color:#38bdf8;">--.- °C</div>
    </div>
    <div class="card">
      <div class="card-label">Pack Voltage</div>
      <div id="v" class="card-val">-- V</div>
    </div>
    <div class="card">
      <div class="card-label">Pack Current</div>
      <div id="i" class="card-val">0 mA</div>
    </div>
    <div class="card">
      <div class="card-label">Risk Index</div>
      <div id="rri" class="card-val">0.00</div>
    </div>
    <div class="card" style="grid-column: 1 / -1;">
      <div class="card-label">Relay Isolation State</div>
      <div id="relay_state" class="card-val" style="color:var(--safe); font-size: 2rem;">CONNECTED (ON)</div>
    </div>
  </div>

  <div class="controls">
    <button class="btn btn-trip" onclick="sendCmd('RELAY_OFF')">TRIP RELAY (ISOLATE)</button>
    <button class="btn btn-on" onclick="sendCmd('RELAY_ON')">ENGAGE RELAY (CLOSE)</button>
  </div>

  <script>
    var ws = new WebSocket(`ws://${location.hostname}:81/`);

    function sendCmd(cmd) {
      ws.send(cmd);
    }

    ws.onmessage = function(e) {
      var d = JSON.parse(e.data);
      document.getElementById('t').innerText = d.t.toFixed(1) + " °C";
      document.getElementById('v').innerText = d.v.toFixed(2) + " V";
      document.getElementById('i').innerText = d.i + " mA";
      document.getElementById('rri').innerText = d.rri.toFixed(2);

      var b = document.getElementById('statusBadge');
      var rText = document.getElementById('relay_state');

      if (d.trip == 1) {
        b.innerText = "TRIPPED: ISOLATED";
        b.style.background = "var(--danger)";
        rText.innerText = "ISOLATED (OFF)";
        rText.style.color = "var(--danger)";
      } else {
        rText.innerText = "CONNECTED (ON)";
        rText.style.color = "var(--safe)";
        if (d.rri > 0.45) {
          b.innerText = "WARNING: HIGH RISK";
          b.style.background = "var(--warn)";
        } else {
          b.innerText = "NORMAL";
          b.style.background = "var(--safe)";
        }
      }
    };
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String cmd = "";
    for (size_t i = 0; i < length; i++) {
      cmd += (char)payload[i];
    }
    cmd.trim();
    if (cmd == "RELAY_ON") {
      STM_Serial.print("RELAY_ON\n");
    } else if (cmd == "RELAY_OFF") {
      STM_Serial.print("RELAY_OFF\n");
    }
  }
}

void setup() {
  Serial.begin(115200);
  // RX2 = GPIO 16 (from STM32 PA9 TX), TX2 = GPIO 17 (to STM32 PA10 RX)
  STM_Serial.begin(115200, SERIAL_8N1, 16, 17);

  WiFi.softAP(ap_ssid, ap_pass);
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  server.handleClient();
  webSocket.loop();

  if (STM_Serial.available()) {
    String json_data = STM_Serial.readStringUntil('\n');
    json_data.trim();
    if (json_data.startsWith("{") && json_data.endsWith("}")) {
      webSocket.broadcastTXT(json_data);
    }
  }
}