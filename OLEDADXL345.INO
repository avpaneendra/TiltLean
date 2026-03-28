/*
===============================================================================================================
ADXL345 Tilt/Lean Detector — ANATOMICAL SOLE on SSD1306 128×64
- Raw I2C (no Adafruit ADXL345 lib needed — only Wire + SSD1306)
- ±2g range → 256 LSB/g (10-bit full-res off, default range)
- 20-sample startup average sets zero reference
- Real tilt angles via atan2 on gravity vector
- Foot outline rotates (LR) + perspective-skews (FB)
- Lean indicator bars bottom + right edge
- ESP32-safe naming throughout
===============================================================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ── OLED ─────────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── ADXL345 I2C address ───────────────────────────────────────
// SDO/ALT pin LOW  → 0x53 (default, most modules)
// SDO/ALT pin HIGH → 0x1D
#define ADXL_ADDR      0x53

// ── ADXL345 register map ──────────────────────────────────────
#define ADXL_DEVID      0x00   // should read 0xE5
#define ADXL_BW_RATE    0x2C   // output data rate
#define ADXL_POWER_CTL  0x2D   // wake / measure bits
#define ADXL_DATA_FORMAT 0x31  // range, full-res, justify
#define ADXL_DATAX0     0x32   // first of 6 data bytes (X LSB)

// ── Sensitivity: ±2g, 10-bit → 256 LSB/g ─────────────────────
#define ADXL_LSB_PER_G  256.0f

// ── Zero-reference offsets (g units, set at startup) ──────────
float ADXL_OFF_X = 0, ADXL_OFF_Y = 0, ADXL_OFF_Z = 0;

// ── Visual / lean config ──────────────────────────────────────
#define MAX_TILT_DEG    30.0f
#define LEAN_DEG_THRESH  3.0f

// ── Foot centre on OLED ───────────────────────────────────────
#define FOOT_CX  56
#define FOOT_CY  30

// ── Anatomical sole polygon (21 pts, local coords) ────────────
#define SOLE_N 21
const int8_t SOLE_LX[SOLE_N] = {
  -4, -7,-10,-12,-13,-12,-10,-11,
  -9, -5,  0,  5,  9, 11, 12, 13,
  13, 12, 10,  7,  4
};
const int8_t SOLE_LY[SOLE_N] = {
 -26,-24,-20,-14, -6,  2,  8, 14,
  22, 26, 27, 26, 22, 14,  8,  2,
  -6,-14,-20,-24,-26
};

// ── Toes [local x, local y, radius×10] ───────────────────────
#define FOOT_TOE_N 5
const int8_t FOOT_TOE_LX[FOOT_TOE_N] = { -8, -4,  0,  4,  7 };
const int8_t FOOT_TOE_LY[FOOT_TOE_N] = { -29,-30,-30,-29,-27 };
const uint8_t FOOT_TOE_R[FOOT_TOE_N] = {  32, 24, 22, 20, 18 };

// ── Metatarsal pad positions ───────────────────────────────────
const int8_t FOOT_META_LX[FOOT_TOE_N] = { -7, -3,  1,  5,  8 };
const int8_t FOOT_META_LY[FOOT_TOE_N] = { -23,-24,-24,-23,-22 };

// ═════════════════════════════════════════════════════════════
// ADXL345 raw I2C helpers
// ═════════════════════════════════════════════════════════════
void adxlWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t adxlRead8(uint8_t reg) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADXL_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.read();
}

// Read AX, AY, AZ as float g values (burst read 6 bytes)
// ADXL345 stores each axis as int16 little-endian (LSB first)
void adxlReadAccel(float &gx, float &gy, float &gz) {
  Wire.beginTransmission(ADXL_ADDR);
  // Set multi-byte read bit (bit 6) for burst
  Wire.write(ADXL_DATAX0 | 0x40);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADXL_ADDR, (uint8_t)6, (uint8_t)true);
  int16_t rx = (Wire.read() | (Wire.read() << 8));
  int16_t ry = (Wire.read() | (Wire.read() << 8));
  int16_t rz = (Wire.read() | (Wire.read() << 8));
  gx = rx / ADXL_LSB_PER_G;
  gy = ry / ADXL_LSB_PER_G;
  gz = rz / ADXL_LSB_PER_G;
}

// ═════════════════════════════════════════════════════════════
// Drawing helpers (identical to MPU6050 version)
// ═════════════════════════════════════════════════════════════
void oledLine(int x0, int y0, int x1, int y1, bool on = true) {
  int c = on ? SSD1306_WHITE : SSD1306_BLACK;
  int dx = abs(x1-x0), dy = abs(y1-y0);
  int sx = x0<x1?1:-1, sy = y0<y1?1:-1, e = dx-dy;
  while (true) {
    if (x0>=0 && x0<128 && y0>=0 && y0<64)
      display.drawPixel(x0, y0, c);
    if (x0==x1 && y0==y1) break;
    int e2 = 2*e;
    if (e2 > -dy) { e -= dy; x0 += sx; }
    if (e2 <  dx) { e += dx; y0 += sy; }
  }
}

void fillPoly(int *px, int *py, int n, bool on = true) {
  int color = on ? SSD1306_WHITE : SSD1306_BLACK;
  int ymin = 64, ymax = 0;
  for (int i = 0; i < n; i++) {
    ymin = min(ymin, py[i]);
    ymax = max(ymax, py[i]);
  }
  ymin = max(ymin, 0);
  ymax = min(ymax, 63);
  for (int y = ymin; y <= ymax; y++) {
    int xs[32]; int xc = 0;
    for (int i = 0; i < n; i++) {
      int j = (i+1) % n;
      if ((py[i]<=y && py[j]>y) || (py[j]<=y && py[i]>y))
        xs[xc++] = (int)(px[i] + (float)(y-py[i])*(px[j]-px[i])/(py[j]-py[i]));
    }
    for (int a = 0; a < xc-1; a++)
      for (int b = a+1; b < xc; b++)
        if (xs[a]>xs[b]) { int t=xs[a]; xs[a]=xs[b]; xs[b]=t; }
    for (int k = 0; k+1 < xc; k += 2)
      for (int x = max(0,xs[k]); x <= min(127,xs[k+1]); x++)
        display.drawPixel(x, y, color);
  }
}

void footXF(float lx, float ly, float angle, float fbScale,
             int &outX, int &outY) {
  float ly2 = ly * fbScale;
  outX = FOOT_CX + (int)round(lx * cos(angle) - ly2 * sin(angle));
  outY = FOOT_CY + (int)round(lx * sin(angle) + ly2 * cos(angle));
}

void drawSole(float lrNorm, float fbNorm) {
  float angle   = lrNorm * 0.40f;
  float fbScale = 1.0f - fbNorm * 0.22f;

  // Outer sole
  int opx[SOLE_N], opy[SOLE_N];
  for (int i = 0; i < SOLE_N; i++)
    footXF(SOLE_LX[i], SOLE_LY[i], angle, fbScale, opx[i], opy[i]);
  fillPoly(opx, opy, SOLE_N, true);

  // Inner hollow
  int ipx[SOLE_N], ipy[SOLE_N];
  for (int i = 0; i < SOLE_N; i++) {
    float lx = SOLE_LX[i], ly = SOLE_LY[i];
    float mag = sqrt(lx*lx + ly*ly);
    if (mag < 1) mag = 1;
    footXF(lx - lx/mag*3.2f, ly - ly/mag*3.2f,
           angle, fbScale, ipx[i], ipy[i]);
  }
  fillPoly(ipx, ipy, SOLE_N, false);

  // Outline
  for (int i = 0; i < SOLE_N; i++) {
    int j = (i+1) % SOLE_N;
    oledLine(opx[i], opy[i], opx[j], opy[j], true);
  }

  // Toes
  for (int t = 0; t < FOOT_TOE_N; t++) {
    int tx, ty;
    footXF(FOOT_TOE_LX[t], FOOT_TOE_LY[t], angle, fbScale, tx, ty);
    int r = max(1, (int)round(FOOT_TOE_R[t] / 10.0f));
    for (int dy = -r; dy <= r; dy++)
      for (int dx = -r; dx <= r; dx++)
        if (dx*dx+dy*dy <= r*r && tx+dx>=0 && tx+dx<128 && ty+dy>=0 && ty+dy<64)
          display.drawPixel(tx+dx, ty+dy, SSD1306_WHITE);
    display.drawCircle(tx, ty, r, SSD1306_WHITE);
  }

  // Heel pad indent
  { int hx, hy; footXF(0, 21, angle, fbScale, hx, hy);
    display.drawCircle(hx, hy, 3, SSD1306_BLACK); }

  // Metatarsal dots
  for (int m = 0; m < FOOT_TOE_N; m++) {
    int mx, my;
    footXF(FOOT_META_LX[m], FOOT_META_LY[m], angle, fbScale, mx, my);
    display.drawPixel(mx, my, SSD1306_BLACK);
  }
}

void drawLabel(float lrDeg, float fbDeg) {
  String lr = "CNTRL";
  if (lrDeg >  LEAN_DEG_THRESH) lr = "RIGHT";
  if (lrDeg < -LEAN_DEG_THRESH) lr = "LEFT ";
  String fb = "     ";
  if (fbDeg >  LEAN_DEG_THRESH) fb = "FRONT";
  if (fbDeg < -LEAN_DEG_THRESH) fb = "BACK ";
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(98,  0); display.print(lr);
  display.setCursor(98, 10); display.print(fb);
}

void drawBars(float lrNorm, float fbNorm) {
  display.drawLine(4, 60, 92, 60, SSD1306_WHITE);
  display.drawLine(48, 58, 48, 62, SSD1306_WHITE);
  int lrDot = constrain((int)(48 + lrNorm * 42), 6, 90);
  display.fillCircle(lrDot, 60, 2, SSD1306_WHITE);

  display.drawLine(95, 4, 95, 56, SSD1306_WHITE);
  display.drawLine(93, 30, 97, 30, SSD1306_WHITE);
  int fbDot = constrain((int)(30 + fbNorm * 24), 6, 54);
  display.fillCircle(95, fbDot, 2, SSD1306_WHITE);
}

// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  // ── Verify chip identity ──────────────────────────────────
  uint8_t devid = adxlRead8(ADXL_DEVID);
  if (devid != 0xE5) {
    Serial.printf("ADXL345 not found! DEVID=0x%02X (expected 0xE5)\n", devid);
    // Check address — try 0x1D if 0x53 fails (SDO pin high)
  } else {
    Serial.println(F("ADXL345 found OK"));
  }

  // ── Configure ADXL345 ─────────────────────────────────────
  adxlWrite(ADXL_BW_RATE,    0x0A);  // 100 Hz output rate (safe default)
  adxlWrite(ADXL_DATA_FORMAT, 0x00); // ±2g, 10-bit, right-justify
  adxlWrite(ADXL_POWER_CTL,  0x08);  // enter Measure mode
  delay(10);

  // ── Init OLED ─────────────────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 failed")); for(;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(10, 20); display.println(F("Calibrating..."));
  display.setCursor(10, 32); display.println(F("Hold still!"));
  display.display();
  delay(2000);

  // ── 20-sample zero reference ──────────────────────────────
  float sumX = 0, sumY = 0, sumZ = 0, gx, gy, gz;
  for (int i = 0; i < 20; i++) {
    adxlReadAccel(gx, gy, gz);
    sumX += gx; sumY += gy; sumZ += gz;
    delay(50);
  }
  ADXL_OFF_X = sumX / 20.0f;
  ADXL_OFF_Y = sumY / 20.0f;
  ADXL_OFF_Z = sumZ / 20.0f;

  Serial.printf("Offsets GX:%.4f GY:%.4f GZ:%.4f\n",
                ADXL_OFF_X, ADXL_OFF_Y, ADXL_OFF_Z);

  display.clearDisplay();
  display.setCursor(18, 24); display.println(F("Zero Set! Ready."));
  display.display();
  delay(1000);
}

void loop() {
  float gx, gy, gz;
  adxlReadAccel(gx, gy, gz);

  // Apply zero offset to X and Y only
  // GZ keeps raw value — gravity component needed for atan2
  float zx = gx - ADXL_OFF_X;
  float zy = gy - ADXL_OFF_Y;
  float zz = gz;   // raw Z preserves gravity reference

  // ── Real tilt angles ──────────────────────────────────────
  // ADXL345 default orientation (flat on desk):
  //   X → points toward connector end
  //   Y → points left (when connector faces away)
  //   Z → points up (perpendicular to PCB)
  //
  // Roll  = Left/Right lean → rotate around X axis
  // Pitch = Front/Back lean → rotate around Y axis
  float rollDeg  = atan2(zy,  zz) * RAD_TO_DEG;   // LR
  float pitchDeg = atan2(-zx, zz) * RAD_TO_DEG;   // FB

  // Normalise to −1…+1
  float lrNorm = constrain(rollDeg,  -MAX_TILT_DEG, MAX_TILT_DEG) / MAX_TILT_DEG;
  float fbNorm = constrain(pitchDeg, -MAX_TILT_DEG, MAX_TILT_DEG) / MAX_TILT_DEG;

  display.clearDisplay();
  drawSole(lrNorm, fbNorm);
  drawLabel(rollDeg, pitchDeg);
  drawBars(lrNorm, fbNorm);
  display.display();

  Serial.printf("Roll:%6.1f°  Pitch:%6.1f°  |  GX:%.3f GY:%.3f GZ:%.3f\n",
                rollDeg, pitchDeg, zx, zy, zz);

  delay(150);
}