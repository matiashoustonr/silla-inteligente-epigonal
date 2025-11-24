// ======================= Config estable =======================
#define BAR_DELAY_MS        50
#define BAR_STEP_PIX        2
#define SPLASH_SWAP_BYTES   1
// =============================================================


// ======================= Inclusiones =========================
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <stdarg.h>
#include <math.h>
#include "austranet_logo_rgb565.h"  // Logo RGB565 320x170
// =============================================================


// ======================= WiFi / Server =======================
const char* SSID = "Matias";
const char* PASS = "21212121";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
bool wifiConnected = false;

enum WifiState { WIFI_IDLE, WIFI_CONNECTING, WIFI_DONE };
WifiState wifiState = WIFI_IDLE;
uint32_t wifiStartMs = 0;
uint32_t wifiTimeoutMs = 8000;
uint32_t wifiDotLastMs = 0;
// =============================================================


// ======================= Botones T-Display S3 ===============
#define BTN_LEFT        0    // KEY1  -> Reintentar WiFi
#define BTN_RIGHT       14   // KEY2  -> Cambiar pantalla
#define BTN_ACTIVE_LOW  1
uint32_t btnLastMs = 0;
uint8_t  prevL = 1, prevR = 1;
// =============================================================


// ======================= Vibrador ===========================
#define PIN_VIB 43   // GPIO43 para el motor vibrador
// =============================================================


// ======================= FSR / ADC ==========================
// 4 FSR usando 4 pines ADC reales de la LilyGO T-Display S3
// Ajusta estos si cableaste distinto
#define PIN_FSR1  1   // GPIO01
#define PIN_FSR2  2   // GPIO02
#define PIN_FSR3  3   // GPIO03
#define PIN_FSR4 10   // GPIO10

// Para mantener compatibilidad con el TARE/CAL/CLI existente,
// usamos el FSR1 como "sensor principal"
#define PIN_FSR   PIN_FSR1

const float VREF     = 3.3f;
const int   ADC_BITS = 12;
const int   ADC_MAX  = (1 << ADC_BITS) - 1;
float R_FIXED  = 3900.0f;

const int   N_SAMPLES = 16;
const float ALPHA     = 0.25f;
float ema_adc = 0.0f;

float invR_zero = 0.0f;
float scale_kg_per_inv = 1.0f;
bool  calibrated = false;
Preferences prefs;
String cmd;
// =============================================================


// ======================= Consola / TFT =======================
TFT_eSPI tft = TFT_eSPI();
int16_t scrW = 0, scrH = 0;
int16_t curX = 0, curY = 0;
int16_t lineH = 16;
bool    tftReady = false;

enum ScreenMode {
  SCREEN_CONSOLE   = 0,
  SCREEN_MONITOR   = 1    // pantalla de S1..S4
};
ScreenMode screenMode = SCREEN_CONSOLE;

// buffer de consola
const int CONSOLE_MAX_LINES = 240;
String consoleBuf[CONSOLE_MAX_LINES];
int    consoleHead = 0;
int    consoleCount = 0;

// área de consola
int consoleX = 0, consoleY = 0, consoleW = 0, consoleH = 0;
// =============================================================


// ======================= Utilidades consola ==================
void console_set_style_normal() {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextSize(1);
  lineH = tft.fontHeight() + 2;
}

void console_reset_cursor() {
  curX = consoleX; curY = consoleY;
  tft.setCursor(curX, curY);
}

void console_clear() {
  tft.fillScreen(TFT_BLACK);
  console_reset_cursor();
}

void console_push_line(const String& line) {
  consoleBuf[consoleHead] = line;
  consoleHead = (consoleHead + 1) % CONSOLE_MAX_LINES;
  if (consoleCount < CONSOLE_MAX_LINES) consoleCount++;
}

int console_lines_per_screen() {
  int n = consoleH / lineH;
  return (n < 1) ? 1 : n;
}

void console_render_tail() {
  // Siempre restaurar estilo normal al volver a la consola
  console_set_style_normal();
  console_clear();
  const int L = console_lines_per_screen();
  int toShow = (consoleCount < L) ? consoleCount : L;
  int startIndex = (consoleHead - toShow + CONSOLE_MAX_LINES) % CONSOLE_MAX_LINES;

  for (int i = 0; i < toShow; ++i) {
    int idx = (startIndex + i) % CONSOLE_MAX_LINES;
    tft.setCursor(consoleX, consoleY + i * lineH);
    tft.print(consoleBuf[idx]);
  }
  curY = consoleY + toShow * lineH;
  if (curY > consoleY + consoleH - lineH) curY = consoleY + consoleH - lineH;
  tft.setCursor(consoleX, curY);
}

void tft_newline() {
  curY += lineH;
  if (curY > consoleY + consoleH - lineH) {
    console_render_tail();
  } else {
    tft.setCursor(consoleX, curY);
  }
}

void tft_print_raw(const char* s) {
  if (!tftReady) return;
  static String currentLine;

  while (*s) {
    char c = *s++;
    if (c == '\r') continue;

    if (c == '\n') {
      console_push_line(currentLine);
      if (screenMode == SCREEN_CONSOLE) {
        tft.setCursor(consoleX, curY);
        tft.print(currentLine);
        tft_newline();
      }
      currentLine = "";
    } else {
      currentLine += c;
      if (screenMode == SCREEN_CONSOLE) {
        tft.print(c);
      }
    }
  }
}

void LOG(const char* s)   { Serial.print(s);   tft_print_raw(s); }
void LOGLN(const char* s) { Serial.println(s); tft_print_raw(s); tft_print_raw("\n"); }

void LOGF(const char* fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  tft_print_raw(buf);
}
// =============================================================


// =================== FSR helpers (FSR1 para CAL) =============================
float readADCavg() {
  long acc = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    acc += analogRead(PIN_FSR);   // FSR1 para TARE/CAL/CLI
    delayMicroseconds(250);
  }
  return float(acc) / float(N_SAMPLES);
}

float adcToV(float a)  { return (a * VREF) / ADC_MAX; }

float vToRfsr(float v) {
  if (v < 0.001f) return 1e9f;
  return R_FIXED * (VREF / v - 1.0f);
}

float invR(float r) {
  return (r > 0.0f) ? (1.0f / r) : 0.0f;
}

void doTare(uint16_t ms = 2000) {
  LOGLN("TARE (FSR1)...");
  unsigned long t0 = millis();
  long n = 0;
  double accInv = 0.0;
  while (millis() - t0 < ms) {
    float a = readADCavg();
    ema_adc = (n == 0) ? a : (ALPHA * a + (1.0f - ALPHA) * ema_adc);
    float v = adcToV(ema_adc);
    float r = vToRfsr(v);
    accInv += invR(r);
    n++;
  }
  invR_zero = (n > 0) ? float(accInv / n) : 0.0f;
  calibrated = false;
  LOGLN("OK");
}

void saveCal() {
  prefs.begin("fsr", false);
  prefs.putFloat("invR_zero", invR_zero);
  prefs.putFloat("scale", scale_kg_per_inv);
  prefs.putBool("calibrated", calibrated);
  prefs.putFloat("R_FIXED", R_FIXED);
  prefs.end();
  LOGLN("Calibración guardada ✅");
}

void loadCal() {
  prefs.begin("fsr", true);
  invR_zero        = prefs.getFloat("invR_zero", 0.0f);
  scale_kg_per_inv = prefs.getFloat("scale", 1.0f);
  calibrated       = prefs.getBool("calibrated", false);
  R_FIXED          = prefs.getFloat("R_FIXED", R_FIXED);
  prefs.end();
  if (calibrated) {
    LOGLN("Calibración cargada ✅");
    LOGF("invR_zero=%.6f  scale=%.6f  R_FIXED=%.1f\n", invR_zero, scale_kg_per_inv, R_FIXED);
  } else {
    LOGLN("Sin calibración previa.");
  }
}

// Lectura en kg SOLO para FSR1 (como antes, para WS)
float readKg() {
  float a = readADCavg();
  ema_adc = (ema_adc == 0.0f) ? a : (ALPHA * a + (1.0f - ALPHA) * ema_adc);
  float v = adcToV(ema_adc);
  float r = vToRfsr(v);
  float inv_rel = invR(r) - invR_zero;
  if (inv_rel < 0) inv_rel = 0;
  if (!calibrated) return 0.0f;
  float kg = scale_kg_per_inv * inv_rel;
  if (kg < 0) kg = 0;
  return kg;
}

// Lectura en kg genérica para cualquier pin FSR (sin EMA global)
float readKgFromPin(int pin) {
  long acc = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    acc += analogRead(pin);
    delayMicroseconds(250);
  }
  float a = float(acc) / float(N_SAMPLES);
  float v = adcToV(a);
  float r = vToRfsr(v);
  float inv_rel = invR(r) - invR_zero;
  if (inv_rel < 0) inv_rel = 0;
  if (!calibrated) return 0.0f;
  float kg = scale_kg_per_inv * inv_rel;
  if (kg < 0) kg = 0;
  return kg;
}
// =============================================================


// ========================= CLI =============================
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
          float a = readADCavg();
          ema_adc = (ema_adc == 0.0f) ? a : (ALPHA * a + (1.0f - ALPHA) * ema_adc);
          float v = adcToV(ema_adc);
          float r = vToRfsr(v);
          float inv_rel = invR(r) - invR_zero;
          if (inv_rel > 1e-9f) {
            scale_kg_per_inv = kg_ref / inv_rel;
            calibrated = true;
            LOGF("CAL OK. scale=%.6f\n", scale_kg_per_inv);
          } else {
            LOGLN("CAL ERROR: muy poca diferencia, usa más peso.");
          }
        } else {
          LOGLN("Uso: CAL <kg>  ej: CAL 1.000");
        }
      }
      else if (cmd == "SAVE") {
        saveCal();
      }
      else if (cmd == "LOAD") {
        loadCal();
      }
      else if (cmd.length()) {
        LOGLN("Comandos: TARE | CAL <kg> | SAVE | LOAD");
      }
      cmd = "";
    } else {
      cmd += c;
    }
  }
}
// =============================================================


// ================= WiFi no bloqueante =======================
void wifi_begin_attempt(uint32_t timeout_ms = 8000) {
  WiFi.disconnect(true, true);
  delay(50);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);

  wifiConnected = false;
  wifiState     = WIFI_CONNECTING;
  wifiStartMs   = millis();
  wifiTimeoutMs = timeout_ms;
  wifiDotLastMs = 0;

  LOGLN("Reintentando WiFi...");
}

void wifi_task_step() {
  if (wifiState != WIFI_CONNECTING) return;

  uint32_t now = millis();
  if (now - wifiDotLastMs >= 300) {
    wifiDotLastMs = now;
    LOG(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    wifiState = WIFI_DONE;
    LOGLN("");
    LOGF("IP ESP32: %s\n", WiFi.localIP().toString().c_str());
    return;
  }

  if (now - wifiStartMs >= wifiTimeoutMs) {
    wifiConnected = false;
    wifiState = WIFI_DONE;
    LOGLN("");
    LOGLN("WiFi no conectado.");
  }
}
// =============================================================


// =================== Splash + Loading bar ==================
void splash_logo_and_loading() {
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(SPLASH_SWAP_BYTES ? true : false);

  const int x = (tft.width()  - austranet_logo_320x170_w) / 2;
  const int y = (tft.height() - austranet_logo_320x170_h) / 2;

  tft.startWrite();
  tft.pushImage(x, y,
                austranet_logo_320x170_w,
                austranet_logo_320x170_h,
                (const uint16_t*)austranet_logo_320x170);
  tft.endWrite();

  int barW = tft.width() - 80;  if (barW < 60) barW = tft.width() - 20;
  const int barH = 12;
  const int barX = (tft.width() - barW) / 2;
  const int barY = tft.height() - 24;

  tft.drawRect(barX, barY, barW, barH, TFT_DARKGREY);
  for (int i = 0; i <= barW - 2; i += BAR_STEP_PIX) {
    tft.fillRect(barX + 1, barY + 1, i, barH - 2, TFT_CYAN);
    delay(BAR_DELAY_MS);
  }

  console_set_style_normal();
  consoleX = 0; consoleY = 0; consoleW = tft.width(); consoleH = tft.height();
  console_clear();
}
// =============================================================


// ============ Pantalla MONITOR FSR (4 sensores) =============
uint32_t lastFSRscreenMs = 0;

// Dibuja solo el marco/título/labels una vez
void initFSRscreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);

  // Título
  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.println("FSR readings (kg)");

  // Separador
  tft.setTextSize(1);
  tft.drawLine(10, 28, tft.width() - 10, 28, TFT_DARKGREY);

  // Footer
  tft.setCursor(10, tft.height() - 15);
  tft.print("KEY2: volver  |  KEY1: WiFi");
}

// Solo actualiza los valores (S1..S4) sin limpiar toda la pantalla
void drawFSRscreen(float k1, float k2, float k3, float k4) {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextSize(1);

  tft.setCursor(10, 50);
  tft.printf("S1: %5.2f kg   |   S2: %5.2f kg", k1, k2);

  tft.setCursor(10, 80);
  tft.printf("S3: %5.2f kg   |   S4: %5.2f kg", k3, k4);
}

void fsrMonitorTask() {
  if (screenMode != SCREEN_MONITOR) return;

  uint32_t now = millis();
  // Actualiza la pantalla ~5 Hz para que se vea más suave y sin tanto parpadeo
  if (now - lastFSRscreenMs < 200) return;
  lastFSRscreenMs = now;

  float k1 = readKgFromPin(PIN_FSR1);
  float k2 = readKgFromPin(PIN_FSR2);
  float k3 = readKgFromPin(PIN_FSR3);
  float k4 = readKgFromPin(PIN_FSR4);

  drawFSRscreen(k1, k2, k3, k4);
}
// =============================================================


// ============== Toggle pantallas (2 modos) ==================
void toggleScreen() {
  if (screenMode == SCREEN_CONSOLE) {
    screenMode = SCREEN_MONITOR;
    initFSRscreen();
    drawFSRscreen(0,0,0,0);   // primera vez
  }
  else {
    screenMode = SCREEN_CONSOLE;
    console_render_tail();
  }
}
// =============================================================


// ======================= Setup / Loop ======================
void setup() {
  // 🔹 Vibrador: APAGARLO LO PRIMERO DE TODO
  digitalWrite(PIN_VIB, LOW);   // Prepara el latch en LOW
  pinMode(PIN_VIB, OUTPUT);     // Configura como salida (ya en LOW)

  // ================== TFT / Consola ==================
  tft.init();
  delay(150);
  tft.setRotation(1);
  tft.invertDisplay(false);
  tft.fillScreen(TFT_BLACK);

  console_set_style_normal();
  consoleX = 0; 
  consoleY = 0; 
  consoleW = tft.width(); 
  consoleH = tft.height();
  console_clear();

  scrW = tft.width();
  scrH = tft.height();
  tftReady = true;

  // Splash con barra de carga
  splash_logo_and_loading();

  // ================== Botones ==================
  pinMode(BTN_LEFT,  BTN_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(BTN_RIGHT, BTN_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  prevL = digitalRead(BTN_LEFT);
  prevR = digitalRead(BTN_RIGHT);

  // (Por seguridad extra, lo dejamos apagado de nuevo,
  //  pero ya debería estar en LOW desde el inicio)
  digitalWrite(PIN_VIB, LOW);

  // ================== Serial / Mensajes ==================
  Serial.begin(115200);
  LOGLN("");
  LOGLN("FSR → kg (TARE/CAL/SAVE/LOAD, monitor 4 FSR, vibrador apagado)");
  LOGLN("KEY2: Consola ↔ Monitor FSR");
  LOGLN("KEY1: Reintentar WiFi");

  // ================== FSR / Calibración ==================
  analogReadResolution(ADC_BITS);
  doTare(2000);   // TARE del FSR1 (puedes omitir y solo usar LOAD si ya calibraste antes)
  loadCal();

  // ================== WiFi / WebSocket ==================
  wifi_begin_attempt(8000);

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType, void*, uint8_t*, size_t){});
  server.addHandler(&ws);
  server.begin();
  LOGLN("Servidor WS listo en /ws");
}


void loop() {
  uint32_t now = millis();

  // Botones
  if (now - btnLastMs > 60) {
    uint8_t sL = digitalRead(BTN_LEFT);
    uint8_t sR = digitalRead(BTN_RIGHT);
    bool pressedL = BTN_ACTIVE_LOW ? (prevL == HIGH && sL == LOW) : (prevL == LOW && sL == HIGH);
    bool pressedR = BTN_ACTIVE_LOW ? (prevR == HIGH && sR == LOW) : (prevR == LOW && sR == HIGH);
    if (pressedR) {
      toggleScreen();
      btnLastMs = now;
    } else if (pressedL) {
      wifi_begin_attempt(8000);
      btnLastMs = now;
    }
    prevL = sL; prevR = sR;
  }

  handleCLI();
  wifi_task_step();
  fsrMonitorTask();   // Actualiza la pantalla de S1..S4

  static unsigned long tSend = 0;
  if (wifiConnected && (millis() - tSend > 50)) {
    tSend = millis();
    float kg = readKg();   // FSR1 por compatibilidad con WS
    char msg[32];
    snprintf(msg, sizeof(msg), "{\"kg\":%.3f}", kg);
    ws.textAll(msg);
  }
}
