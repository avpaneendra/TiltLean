/*
===============================================================================================================
BMP280 + SSD1306 OLED — Test Sketch
Verifies I2C comms for both chips, displays:
  - Temperature (°C)
  - Pressure (hPa)
  - Altitude estimate (m)
Live refresh every 500ms. Serial mirror for debugging.
===============================================================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>

// ── OLED ─────────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDR      0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── BMP280 ────────────────────────────────────────────────────
// SDO LOW  → 0x76 (most modules)
// SDO HIGH → 0x77
#define BMP_ADDR       0x76
Adafruit_BMP280 bmp;

// ── Sea-level pressure reference for altitude calc (hPa) ──────
#define SEA_LEVEL_HPA  1013.25f

// ── Simple horizontal bar (for pressure visual) ───────────────
void drawBar(int x, int y, int w, int h, float fraction) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int fill = constrain((int)(fraction * (w - 2)), 0, w - 2);
  if (fill > 0)
    display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  // ── Init OLED ─────────────────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 not found — check wiring / address"));
    for (;;);
  }
  Serial.println(F("SSD1306 OK"));

  // ── Splash ────────────────────────────────────────────────
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20,  8); display.println(F("BMP280 + OLED"));
  display.setCursor(28, 22); display.println(F("Test Sketch"));
  display.setCursor(16, 38); display.println(F("Checking sensor..."));
  display.display();
  delay(1500);

  // ── Init BMP280 ───────────────────────────────────────────
  if (!bmp.begin(BMP_ADDR)) {
    // Show error on OLED and halt
    display.clearDisplay();
    display.setCursor(0,  0); display.println(F("BMP280 NOT FOUND!"));
    display.setCursor(0, 14); display.println(F("Check:"));
    display.setCursor(0, 24); display.println(F(" - Wiring (SDA/SCL)"));
    display.setCursor(0, 34); display.println(F(" - Address 0x76/0x77"));
    display.setCursor(0, 44); display.println(F(" - 3.3V power"));
    display.display();
    Serial.println(F("BMP280 not found — try address 0x77"));
    for (;;);
  }
  Serial.println(F("BMP280 OK"));

  // ── BMP280 recommended settings for indoor navigation ─────
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,    // temperature oversampling
    Adafruit_BMP280::SAMPLING_X16,   // pressure oversampling
    Adafruit_BMP280::FILTER_X16,     // IIR filter coefficient
    Adafruit_BMP280::STANDBY_MS_500  // standby time
  );

  display.clearDisplay();
  display.setCursor(28, 24); display.println(F("All Good!"));
  display.setCursor(20, 38); display.println(F("Starting..."));
  display.display();
  delay(1000);
}

// ─────────────────────────────────────────────────────────────
void loop() {
  float tempC    = bmp.readTemperature();           // °C
  float pressHpa = bmp.readPressure() / 100.0f;    // Pa → hPa
  float altM     = bmp.readAltitude(SEA_LEVEL_HPA); // metres

  // ── OLED layout ───────────────────────────────────────────
  //
  //  TEMP          PRESS
  //  ##.#°C        ####.# hPa
  //  ──────────────────────
  //  ALT
  //  ####.# m
  //  [=========         ]   ← pressure bar 900–1100 hPa
  //
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // ── Row 1: labels ────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(0,  0); display.print(F("TEMP"));
  display.setCursor(70, 0); display.print(F("PRESS"));

  // ── Row 2: values (large) ─────────────────────────────────
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(tempC, 1);
  display.print(F("C"));

  display.setTextSize(1);
  display.setCursor(70, 10);
  display.print(pressHpa, 1);
  display.setCursor(70, 20);
  display.print(F("hPa"));

  // ── Divider ───────────────────────────────────────────────
  display.drawLine(0, 30, 127, 30, SSD1306_WHITE);

  // ── Row 3: altitude ───────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(0, 33); display.print(F("ALT"));
  display.setTextSize(2);
  display.setCursor(0, 42);
  display.print(altM, 1);
  display.setTextSize(1);
  display.setCursor(80, 49); display.print(F("m"));

  // ── Pressure bar (900–1100 hPa range) ─────────────────────
  float barFrac = (pressHpa - 900.0f) / 200.0f;   // 900→0.0, 1100→1.0
  drawBar(0, 57, 128, 7, barFrac);

  display.display();

  // ── Serial mirror ─────────────────────────────────────────
  Serial.printf("Temp: %.2f C  |  Press: %.2f hPa  |  Alt: %.2f m\n",
                tempC, pressHpa, altM);

  delay(500);
}
