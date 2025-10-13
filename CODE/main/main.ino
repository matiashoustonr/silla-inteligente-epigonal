#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// ===== WiFi =====
const char* SSID = "Matias";
const char* PASS = "21212121";

// ===== WebSocket =====
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===== FSR (ADC1, GPIO 4) =====
#define PIN_FSR 4
const float VREF   = 3.3f;
const int   ADC_BITS = 12;
const int   ADC_MAX  = (1 << ADC_BITS) - 1;
float R_FIXED  = 3900.0f;      // tu resistencia ~3.9kΩ

// Filtro/lectura
const int   N_SAMPLES = 16;
const float ALPHA     = 0.25f;
float ema_adc = 0.0f;

// Calibración 1 punto (persistente)
float invR_zero = 0.0f;          // cero (1/Ω)
float scale_kg_per_inv = 1.0f;   // kg por (1/Ω)
bool  calibrated = false;
Preferences prefs;
String cmd;

// ---------- Utilidades ----------
float readADCavg() {
  long acc = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    acc += analogRead(PIN_FSR);
    delayMicroseconds(250);
  }
  return float(acc) / float(N_SAMPLES);
}
float adcToV(float a)              { return (a * VREF) / ADC_MAX; }
float vToRfsr(float v)             { if (v < 0.001f) return 1e9f; return R_FIXED * (VREF / v - 1.0f); }
float invR(float r)                { return (r > 0.0f) ? (1.0f / r) : 0.0f; }

// ---------- TARE ----------
void doTare(uint16_t ms = 2000) {
  Serial.print("TARE... ");
  unsigned long t0 = millis();
  long n = 0;
  double accInv = 0.0;
  while (millis() - t0 < ms) {
    float a = readADCavg();
    ema_adc = (n == 0) ? a : (ALPHA * a + (1 - ALPHA) * ema_adc);
    float v = adcToV(ema_adc);
    float r = vToRfsr(v);
    accInv += invR(r);
    n++;
  }
  invR_zero = (n > 0) ? float(accInv / n) : 0.0f;
  calibrated = false;
  Serial.println("OK");
}

// ---------- SAVE / LOAD ----------
void saveCal() {
  prefs.begin("fsr", false);
  prefs.putFloat("invR_zero", invR_zero);
  prefs.putFloat("scale", scale_kg_per_inv);
  prefs.putBool("calibrated", calibrated);
  prefs.putFloat("R_FIXED", R_FIXED);
  prefs.end();
  Serial.println("Calibración guardada ✅");
}
void loadCal() {
  prefs.begin("fsr", true);
  invR_zero        = prefs.getFloat("invR_zero", 0.0f);
  scale_kg_per_inv = prefs.getFloat("scale", 1.0f);
  calibrated       = prefs.getBool("calibrated", false);
  R_FIXED          = prefs.getFloat("R_FIXED", R_FIXED);
  prefs.end();
  if (calibrated) {
    Serial.println("Calibración cargada ✅");
    Serial.printf("invR_zero=%.6f  scale=%.6f  R_FIXED=%.1f\n", invR_zero, scale_kg_per_inv, R_FIXED);
  } else {
    Serial.println("Sin calibración previa.");
  }
}

// ---------- kg en tiempo real ----------
float readKg() {
  float a = readADCavg();
  ema_adc = (ema_adc == 0.0f) ? a : (ALPHA * a + (1 - ALPHA) * ema_adc);
  float v = adcToV(ema_adc);
  float r = vToRfsr(v);
  float inv_rel = invR(r) - invR_zero;
  if (inv_rel < 0) inv_rel = 0;
  if (!calibrated) return 0.0f;
  float kg = scale_kg_per_inv * inv_rel;
  if (kg < 0) kg = 0;
  return kg;
}

// ---------- CLI Serial ----------
void handleCLI() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmd.trim(); cmd.toUpperCase();

      if (cmd == "TARE") {
        doTare(1500);
      }
      else if (cmd.startsWith("CAL ")) {
        int sp = cmd.indexOf(' ');
        float kg_ref = cmd.substring(sp + 1).toFloat();
        if (kg_ref > 0.0f) {
          // forzar una actualización del filtro
          float a = readADCavg();
          ema_adc = (ema_adc == 0.0f) ? a : (ALPHA * a + (1 - ALPHA) * ema_adc);
          float v = adcToV(ema_adc);
          float r = vToRfsr(v);
          float inv_rel = invR(r) - invR_zero;
          if (inv_rel > 1e-9f) {
            scale_kg_per_inv = kg_ref / inv_rel;
            calibrated = true;
            Serial.printf("CAL OK. scale=%.6f\n", scale_kg_per_inv);
          } else {
            Serial.println("CAL ERROR: muy poca diferencia, usa más peso.");
          }
        } else {
          Serial.println("Uso: CAL <kg>  ej: CAL 1.000");
        }
      }
      else if (cmd == "SAVE") {
        saveCal();
      }
      else if (cmd == "LOAD") {
        loadCal();
      }
      else if (cmd.length()) {
        Serial.println("Comandos: TARE | CAL <kg> | SAVE | LOAD");
      }
      cmd = "";
    } else {
      cmd += c;
    }
  }
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  analogReadResolution(ADC_BITS);

  Serial.println("\nFSR → kg (WebSocket + TARE/CAL/SAVE/LOAD)");
  doTare(2000);
  loadCal();

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nIP ESP32: %s\n", WiFi.localIP().toString().c_str());

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType, void*, uint8_t*, size_t){});
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  handleCLI();

  static unsigned long t = 0;
  if (millis() - t > 50) { // ~20 Hz
    t = millis();
    float kg = readKg();
    char msg[32];
    snprintf(msg, sizeof(msg), "{\"kg\":%.3f}", kg);
    ws.textAll(msg);
  }
}
