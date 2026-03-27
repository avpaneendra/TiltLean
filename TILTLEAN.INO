/*
===============================================================================================================
GY-271 (QMC5883L) Tilt/Lean Detector with OLED SSD1306 Display
- Reads XYZ on startup and sets as zero reference (offset calibration)
- Detects Left/Right lean and Front/Back lean from relative changes
- Displays lean direction and angle on 128x64 OLED via I2C
===============================================================================================================
*/

#include <QMC5883LCompass.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED Config ──────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1        // No reset pin
#define SCREEN_ADDRESS 0x3C    // Common SSD1306 I2C address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
QMC5883LCompass compass;

// ── Zero-reference offsets (set at startup) ──────────────────
int offsetX = 0;
int offsetY = 0;
int offsetZ = 0;

// ── Threshold to avoid noise triggering direction labels ──────
#define LEAN_THRESHOLD 50   // Adjust based on sensitivity needed

void setup() {
  Serial.begin(9600);
  compass.init();

  // ── Init OLED ─────────────────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);  // Halt
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(F("  Calibrating..."));
  display.println(F("  Hold still!"));
  display.display();
  delay(2000);

  // ── Capture startup position as ZERO reference ────────────
  // Average multiple readings for stability
  long sumX = 0, sumY = 0, sumZ = 0;
  int samples = 20;

  for (int i = 0; i < samples; i++) {
    compass.read();
    sumX += compass.getX();
    sumY += compass.getY();
    sumZ += compass.getZ();
    delay(50);
  }

  // Arithmetic zero-offset: subtract startup average from all future readings
  offsetX = sumX / samples;
  offsetY = sumY / samples;
  offsetZ = sumZ / samples;

  Serial.print("Offsets -> X: "); Serial.print(offsetX);
  Serial.print(" Y: "); Serial.print(offsetY);
  Serial.print(" Z: "); Serial.println(offsetZ);

  display.clearDisplay();
  display.setCursor(0, 20);
  display.println(F("  Zero Set!"));
  display.println(F("  Ready."));
  display.display();
  delay(1000);
}

void loop() {
  // ── Read raw sensor values ────────────────────────────────
  compass.read();
  int rawX = compass.getX();
  int rawY = compass.getY();
  int rawZ = compass.getZ();

  // ── Apply zero-offset (arithmetic subtraction) ────────────
  int x = rawX - offsetX;   // Zeroed X
  int y = rawY - offsetY;   // Zeroed Y
  int z = rawZ - offsetZ;   // Zeroed Z

  // ── Determine Left/Right lean (based on Y axis) ───────────
  // Y positive = Right lean, Y negative = Left lean
  String lrDirection = "LEVEL";
  int lrMagnitude = abs(y);

  if (y > LEAN_THRESHOLD)        lrDirection = "RIGHT";
  else if (y < -LEAN_THRESHOLD)  lrDirection = "LEFT";

  // ── Determine Front/Back lean (based on X axis) ───────────
  // X positive = Forward lean, X negative = Backward lean
  String fbDirection = "CENTER";
  int fbMagnitude = abs(x);

  if (x > LEAN_THRESHOLD)        fbDirection = "FRONT";
  else if (x < -LEAN_THRESHOLD)  fbDirection = "BACK";

  // ── Serial Debug Output ───────────────────────────────────
  Serial.print("Zeroed -> X: "); Serial.print(x);
  Serial.print(" Y: "); Serial.print(y);
  Serial.print(" Z: "); Serial.print(z);
  Serial.print("  |  LR: "); Serial.print(lrDirection);
  Serial.print(" ("); Serial.print(lrMagnitude); Serial.print(")");
  Serial.print("  FB: "); Serial.print(fbDirection);
  Serial.print(" ("); Serial.print(fbMagnitude); Serial.println(")");

  // ── Update OLED Display ───────────────────────────────────
  display.clearDisplay();

  // Title bar
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("-- LEAN DETECTOR --"));

  // Divider line
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  // Left/Right section
  display.setCursor(0, 13);
  display.print(F("L/R : "));
  display.setTextSize(1);
  display.println(lrDirection);
  display.setCursor(0, 23);
  display.print(F("  Mag: "));
  display.println(lrMagnitude);

  // Front/Back section
  display.setCursor(0, 35);
  display.print(F("F/B : "));
  display.println(fbDirection);
  display.setCursor(0, 45);
  display.print(F("  Mag: "));
  display.println(fbMagnitude);

  // Raw zeroed Z value at bottom
  display.drawLine(0, 54, 127, 54, SSD1306_WHITE);
  display.setCursor(0, 56);
  display.print(F("Z: "));
  display.print(z);

  display.display();

  delay(250);
}
