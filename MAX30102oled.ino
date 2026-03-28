/**
 @file Max30102_OLED.ino
 @brief Heart Rate and SpO2 with SSD1306 OLED display
 @detail Original MAX30102 logic preserved.
         ESP32 fix: removed TwoWire Wire(0) — use built-in Wire directly.
 
 * Connections (shared I2C bus):
 * MAX30102 / OLED    ESP32
 * 3V3            -   3.3V
 * GND            -   GND
 * SDA            -   GPIO 21
 * SCL            -   GPIO 22
 * Use 10K pull-up resistors on SDA and SCL.
 **/

#include <Wire.h>
// ── REMOVED: TwoWire Wire(0); ─────────────────────────────────
// ESP32 core already defines Wire globally in Wire.cpp.
// Declaring it again causes "multiple definition" linker error.
// If you need I2C port 1: use Wire1 (built-in) or Wire.begin(SDA,SCL)

#include "MAX30102_PulseOximeter.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED ─────────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDR      0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── MAX30102 ──────────────────────────────────────────────────
#define REPORTING_PERIOD_MS  1000
PulseOximeter pox;
uint32_t tsLastReport = 0;

// ── Beat flash ────────────────────────────────────────────────
bool     BEAT_FLASH = false;
uint32_t BEAT_TIME  = 0;
#define  FLASH_MS   150

// ── Waveform buffer ───────────────────────────────────────────
#define  WAVE_LEN   128
uint8_t  WAVE_BUF[WAVE_LEN];
uint8_t  WAVE_HEAD  = 0;

// ── Last valid readings ───────────────────────────────────────
float    LAST_BPM   = 0;
float    LAST_SPO2  = 0;

// ─────────────────────────────────────────────────────────────
void onBeatDetected() {
  Serial.println("Beat!");
  BEAT_FLASH       = true;
  BEAT_TIME        = millis();
  WAVE_BUF[WAVE_HEAD] = 0;
  WAVE_HEAD        = (WAVE_HEAD + 1) % WAVE_LEN;
}

void wavePushIdle() {
  static uint8_t phase = 0;
  uint8_t y = 6 + (uint8_t)(4.0f * sin(phase * 0.25f));
  WAVE_BUF[WAVE_HEAD] = y;
  WAVE_HEAD = (WAVE_HEAD + 1) % WAVE_LEN;
  phase++;
}

void drawWave(int zoneY) {
  for (int x = 0; x < WAVE_LEN; x++) {
    int idx = (WAVE_HEAD + x) % WAVE_LEN;
    int py  = zoneY + WAVE_BUF[idx];
    if (py >= 0 && py < SCREEN_HEIGHT)
      display.drawPixel(x, py, SSD1306_WHITE);
    if (py + 1 < SCREEN_HEIGHT)
      display.drawPixel(x, py + 1, SSD1306_WHITE);
  }
}

void drawHeart(int x, int y, bool filled) {
  if (filled) {
    display.fillCircle(x + 3, y + 2, 3, SSD1306_WHITE);
    display.fillCircle(x + 9, y + 2, 3, SSD1306_WHITE);
    display.fillTriangle(x, y + 3, x + 12, y + 3,
                         x + 6, y + 10, SSD1306_WHITE);
  } else {
    display.drawCircle(x + 3, y + 2, 3, SSD1306_WHITE);
    display.drawCircle(x + 9, y + 2, 3, SSD1306_WHITE);
    display.drawLine(x,      y + 3, x + 6, y + 10, SSD1306_WHITE);
    display.drawLine(x + 12, y + 3, x + 6, y + 10, SSD1306_WHITE);
  }
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // ── Explicit I2C pin init for ESP32 ───────────────────────
  // Default ESP32: SDA=21, SCL=22
  // Change pins here if your board is wired differently:
  Wire.begin(21, 22);

  // ── OLED ──────────────────────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 not found"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(16,  8); display.println(F("MAX30102 + OLED"));
  display.setCursor(24, 24); display.println(F("Initializing.."));
  display.display();

  Serial.print("Initializing..");
  delay(3000);

  // ── MAX30102 ──────────────────────────────────────────────
  if (!pox.begin()) {
    Serial.println("FAILED");
    display.clearDisplay();
    display.setCursor(0,  0); display.println(F("MAX30102 FAILED!"));
    display.setCursor(0, 14); display.println(F("Check wiring:"));
    display.setCursor(0, 26); display.println(F(" SDA=21 / SCL=22"));
    display.setCursor(0, 38); display.println(F(" 10K pull-ups"));
    display.setCursor(0, 50); display.println(F(" 3.3V power only"));
    display.display();
    for (;;);
  }
  Serial.println("SUCCESS");

  pox.setIRLedCurrent(MAX30102_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  memset(WAVE_BUF, 7, WAVE_LEN);

  display.clearDisplay();
  display.setCursor(22, 20); display.println(F("Sensor Ready!"));
  display.setCursor(12, 34); display.println(F("Place finger now"));
  display.display();
  delay(1000);
}

// ─────────────────────────────────────────────────────────────
void loop() {
  pox.update();   // must stay first, called as fast as possible

  if (BEAT_FLASH && millis() - BEAT_TIME > FLASH_MS)
    BEAT_FLASH = false;

  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {

    float bpm  = pox.getHeartRate();
    float spo2 = pox.getSpO2();

    if (bpm  > 0) LAST_BPM  = bpm;
    if (spo2 > 0) LAST_SPO2 = spo2;

    wavePushIdle();

    // ── Serial ────────────────────────────────────────────
    Serial.print("Heart rate:");
    Serial.print(bpm);
    Serial.print("bpm / SpO2:");
    Serial.print(spo2);
    Serial.println("%");

    tsLastReport = millis();

    // ── OLED ──────────────────────────────────────────────
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    bool fingerOn = (spo2 > 0);

    if (!fingerOn) {
      display.setTextSize(1);
      display.setCursor(22,  4); display.println(F("MAX30102 Ready"));
      display.drawLine(0, 14, 127, 14, SSD1306_WHITE);
      display.setCursor(16, 20); display.println(F("Place finger on"));
      display.setCursor(28, 30); display.println(F("the sensor to"));
      display.setCursor(28, 40); display.println(F("begin reading"));
      static uint8_t dotPhase = 0;
      dotPhase = (dotPhase + 1) % 4;
      for (int d = 0; d < dotPhase; d++)
        display.fillCircle(48 + d * 10, 56, 2, SSD1306_WHITE);

    } else {
      drawHeart(0, 0, BEAT_FLASH);

      display.setTextSize(1);
      display.setCursor(18,  0); display.print(F("HEART RATE"));
      display.setTextSize(2);
      display.setCursor(0, 12);
      if (LAST_BPM > 0) display.print((int)LAST_BPM);
      else               display.print(F("--"));
      display.setTextSize(1);
      display.setCursor(0, 30); display.print(F("BPM"));

      display.setTextSize(1);
      display.setCursor(72,  0); display.print(F("SpO2"));
      display.setTextSize(2);
      display.setCursor(68, 12);
      if (LAST_SPO2 > 0) {
        display.print((int)LAST_SPO2);
        display.setTextSize(1);
        display.setCursor(112, 20); display.print(F("%"));
      } else {
        display.print(F("--"));
      }

      display.setTextSize(1);
      display.setCursor(68, 30);
      if      (LAST_SPO2 >= 95) display.print(F("Normal"));
      else if (LAST_SPO2 >= 90) display.print(F("Low   "));
      else if (LAST_SPO2 >  0)  display.print(F("Alert!"));

      display.drawLine(0, 38, 127, 38, SSD1306_WHITE);
      drawWave(40);
    }

    display.display();
  }
}
