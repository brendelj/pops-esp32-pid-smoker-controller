/*
  Pop's PID Smoker Controller - ESP32 Companion Bridge

  Split-project architecture:
    - Arduino Nano owns PID, outputs, probes, local LCD/menu, and safety.
    - ESP32 owns Wi-Fi, web dashboard/API, SD logging, and remote commands.

  Hardware target:
    - ESP32 DevKit / WROOM32
    - UART2 to Nano serial pins
    - Optional SD card on VSPI

  Wiring notes:
    ESP32 TX2 GPIO17 -> Nano RX/D0 through level shifting/divider when needed
    ESP32 RX2 GPIO16 <- Nano TX/D1
    SD CS=5, SCK=18, MISO=19, MOSI=23

  Nano telemetry expected, newline terminated JSON:
    {"tmpC":107.2,"tgtC":107.2,"p1C":66.1,"p2C":0,"heatPct":45,"coolPct":0,"servoDeg":0,"state":"RUN","alarm":"OK"}
*/

#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <ESPmDNS.h>

// ---------------- Wi-Fi ----------------
#define STA_MODE 0
const char* AP_SSID = "PopsPID";
const char* AP_PASS = "controller123";
const char* STA_SSID = "your-ssid";
const char* STA_PASS = "your-pass";

// ---------------- Pins ----------------
const int PIN_SD_CS   = 5;
const int PIN_SD_SCK  = 18;
const int PIN_SD_MISO = 19;
const int PIN_SD_MOSI = 23;
const int UART_RX2    = 16;
const int UART_TX2    = 17;

// ---------------- Globals ----------------
WebServer server(80);
HardwareSerial NanoSerial(2);
bool sdOk = false;
String lastTelemetry = "";
String lastNanoLine = "";
unsigned long lastTelemetryMs = 0;
unsigned long bootMs = 0;

// ---------------- Helpers ----------------
String jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\' || c == '"') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

void sendToNano(const String& line) {
  NanoSerial.println(line);
}

void appendLog(const String& line) {
  if (!sdOk) return;
  File f = SD.open("/pops_pid_log.csv", FILE_APPEND);
  if (!f) return;
  f.print(millis());
  f.print(',');
  f.println(line);
  f.close();
}

String ipText() {
  return WiFi.getMode() == WIFI_AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
}

String pageHeader() {
  String h;
  h += F("<!doctype html><html><head><meta charset='utf-8'>");
  h += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  h += F("<title>Pop's PID</title>");
  h += F("<style>body{font-family:system-ui,Arial;margin:24px;max-width:900px}pre{background:#f4f4f4;padding:12px;border-radius:8px;overflow:auto}.card{border:1px solid #ddd;border-radius:8px;padding:12px;margin:10px 0}button,input{font-size:1rem;padding:8px;margin:4px}</style>");
  h += F("<script>async function refresh(){let r=await fetch('/api/status');let j=await r.json();document.getElementById('raw').textContent=JSON.stringify(j,null,2);}async function cmd(u){await fetch(u);refresh();}setInterval(refresh,2000);window.onload=refresh;</script>");
  h += F("</head><body><h1>Pop's PID Smoker Controller</h1>");
  return h;
}

String pageFooter() {
  return F("</body></html>");
}

// ---------------- HTTP ----------------
void handleRoot() {
  String h = pageHeader();
  h += F("<div class='card'><b>Bridge IP:</b> "); h += ipText();
  h += F("<br><b>SD:</b> "); h += sdOk ? "OK" : "not present";
  h += F("<br><b>Uptime:</b> "); h += String((millis() - bootMs) / 1000); h += F(" seconds</div>");

  h += F("<div class='card'><h3>Controls</h3>");
  h += F("<button onclick=\"cmd('/api/start')\">Start</button>");
  h += F("<button onclick=\"cmd('/api/stop')\">Stop</button>");
  h += F("<form onsubmit=\"cmd('/api/set?tgtF='+document.getElementById('tgt').value);return false;\">");
  h += F("Target °F: <input id='tgt' type='number' value='225'><button>Set</button></form>");
  h += F("</div>");

  h += F("<div class='card'><h3>Live Status</h3><pre id='raw'>Loading...</pre></div>");
  h += F("<div class='card'><a href='/log'>Download log</a> | <a href='/tail'>Tail log</a></div>");
  h += pageFooter();
  server.send(200, "text/html", h);
}

void handleApiStatus() {
  String j = "{";
  j += "\"ip\":\"" + ipText() + "\",";
  j += "\"sdOk\":" + String(sdOk ? "true" : "false") + ",";
  j += "\"uptimeSec\":" + String((millis() - bootMs) / 1000) + ",";
  j += "\"lastTelemetryAgeMs\":" + String(lastTelemetryMs ? millis() - lastTelemetryMs : 0) + ",";
  j += "\"raw\":\"" + jsonEscape(lastTelemetry) + "\"";
  j += "}";
  server.send(200, "application/json", j);
}

void handleStart() {
  sendToNano("{\"cmd\":\"start\"}");
  server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"start\"}");
}

void handleStop() {
  sendToNano("{\"cmd\":\"stop\"}");
  server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"stop\"}");
}

void handleSet() {
  if (server.hasArg("tgtF")) {
    sendToNano(String("{\"cmd\":\"set\",\"tgtF\":") + server.arg("tgtF") + "}");
  } else if (server.hasArg("tgtC")) {
    sendToNano(String("{\"cmd\":\"set\",\"tgtC\":") + server.arg("tgtC") + "}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing tgtF or tgtC\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handlePid() {
  String cmd = "{\"cmd\":\"pid\"";
  if (server.hasArg("kp")) cmd += ",\"kp\":" + server.arg("kp");
  if (server.hasArg("ki")) cmd += ",\"ki\":" + server.arg("ki");
  if (server.hasArg("kd")) cmd += ",\"kd\":" + server.arg("kd");
  cmd += "}";
  sendToNano(cmd);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTail() {
  if (!sdOk) { server.send(404, "text/plain", "SD not present"); return; }
  File f = SD.open("/pops_pid_log.csv", FILE_READ);
  if (!f) { server.send(404, "text/plain", "No log yet"); return; }
  const size_t maxBytes = 8192;
  if (f.size() > maxBytes) f.seek(f.size() - maxBytes);
  String out;
  while (f.available()) out += (char)f.read();
  f.close();
  server.send(200, "text/plain", out);
}

void handleLog() {
  if (!sdOk || !SD.exists("/pops_pid_log.csv")) {
    server.send(404, "text/plain", "No log available");
    return;
  }
  File f = SD.open("/pops_pid_log.csv", FILE_READ);
  server.streamFile(f, "text/csv");
  f.close();
}

void handleNotFound() {
  server.send(404, "application/json", "{\"ok\":false,\"error\":\"not found\"}");
}

// ---------------- Setup ----------------
void setupWiFi() {
#if STA_MODE
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) delay(250);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
  }
#else
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
#endif
  MDNS.begin("popspid");
}

void setupSd() {
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  sdOk = SD.begin(PIN_SD_CS);
  if (sdOk && !SD.exists("/pops_pid_log.csv")) {
    File f = SD.open("/pops_pid_log.csv", FILE_WRITE);
    if (f) {
      f.println("millis,json");
      f.close();
    }
  }
}

void setupRoutes() {
  server.on("/", handleRoot);
  server.on("/api/status", handleApiStatus);
  server.on("/api/start", handleStart);
  server.on("/api/stop", handleStop);
  server.on("/api/set", handleSet);
  server.on("/api/pid", handlePid);
  server.on("/tail", handleTail);
  server.on("/log", handleLog);
  server.onNotFound(handleNotFound);
  server.begin();
}

void readNano() {
  while (NanoSerial.available()) {
    char c = (char)NanoSerial.read();
    if (c == '\n') {
      lastNanoLine.trim();
      if (lastNanoLine.length()) {
        lastTelemetry = lastNanoLine;
        lastTelemetryMs = millis();
        appendLog(lastTelemetry);
      }
      lastNanoLine = "";
    } else if (lastNanoLine.length() < 512) {
      lastNanoLine += c;
    }
  }
}

void setup() {
  bootMs = millis();
  Serial.begin(115200);
  NanoSerial.begin(115200, SERIAL_8N1, UART_RX2, UART_TX2);
  setupWiFi();
  setupSd();
  setupRoutes();
  Serial.print("Pop's PID bridge ready at ");
  Serial.println(ipText());
}

void loop() {
  readNano();
  server.handleClient();
}
