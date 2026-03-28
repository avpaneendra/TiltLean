/*
===============================================================================================================
MPU6050 Tilt/Lean Detector — ANATOMICAL SOLE on SSD1306 128×64
- Uses accelerometer gravity vector → real tilt angles via atan2
- Startup average (20 samples) sets zero reference
- Foot outline rotates (LR) + perspective-skews (FB) on OLED
- Lean indicator bars on bottom + right edge
- ESP32-safe naming (no TX/RX/MX/MY conflicts)
===============================================================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ── OLED Config ──────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── MPU6050 I2C address ───────────────────────────────────────
#define MPU_ADDR       0x68   // AD0 low; use 0x69 if AD0 pulled high

// ── Zero-reference offsets (accel raw, set at startup) ────────
float offsetAX = 0, offsetAY = 0, offsetAZ = 0;

// ── Max tilt angle mapped to full visual rotation (degrees) ───
#define MAX_TILT_DEG   30.0f

// ── Lean threshold in degrees (noise gate for label) ──────────
#define LEAN_DEG_THRESH  3.0f

// ── Foot centre on OLED ───────────────────────────────────────
#define FOOT_CX  56
#define FOOT_CY  30

// ── Anatomical sole polygon (21 pts, local coords) ────────────
// Toes = Y−, Heel = Y+, BigToe side = X−, LittleToe = X+
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

// ── Toe circles [local x, local y, radius×10] ─────────────────
#define FOOT_TOE_N 5
const int8_t FOOT_TOE_LX[FOOT_TOE_N] = { -8, -4,  0,  4,  7 };
const int8_t FOOT_TOE_LY[FOOT_TOE_N] = { -29,-30,-30,-29,-27 };
const uint8_t FOOT_TOE_R[FOOT_TOE_N]  = {  32, 24, 22, 20, 18 };

// ── Metatarsal pad positions ──────────────────────────────────
const int8_t FOOT_META_LX[FOOT_TOE_N] = { -7, -3,  1,  5,  8 };
const int8_t FOOT_META_LY[FOOT_TOE_N] = { -23,-24,-24,-23,-22 };

// ═════════════════════════════════════════════════════════════
// MPU6050 helpers (raw I2C, no extra library needed)
// ═════════════════════════════════════════════════════════════
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

// Read 3 × int16 from consecutive registers (AX, AY, AZ or GX,GY,GZ)
void mpuRead3(uint8_t startReg, float &a, float &b, float &c) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
  int16_t ra = Wire.read()<<8 | Wire.read();
  int16_t rb = Wire.read()<<8 | Wire.read();
  int16_t rc = Wire.read()<<8 | Wire.read();
  // ±2g range → 16384 LSB/g
  a = ra / 16384.0f;
  b = rb / 16384.0f;
  c = rc / 16384.0f;
}

// ═════════════════════════════════════════════════════════════
// Drawing helpers
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

// Transform local foot point → OLED screen coords
void footXF(float lx, float ly, float angle, float fbScale,
             int &outX, int &outY) {
  float ly2 = ly * fbScale;
  outX = FOOT_CX + (int)round(lx * cos(angle) - ly2 * sin(angle));
  outY = FOOT_CY + (int)round(lx * sin(angle) + ly2 * cos(angle));
}

// ── Draw anatomical sole ──────────────────────────────────────
// lrNorm: −1.0 … +1.0  (LEFT … RIGHT)
// fbNorm: −1.0 … +1.0  (FRONT … BACK)
void drawSole(float lrNorm, float fbNorm) {
  float angle   = lrNorm * 0.40f;         // ±23° visual rotation
  float fbScale = 1.0f - fbNorm * 0.22f;  // perspective squish

  // 1. Outer sole polygon
  int opx[SOLE_N], opy[SOLE_N];
  for (int i = 0; i < SOLE_N; i++)
    footXF(SOLE_LX[i], SOLE_LY[i], angle, fbScale, opx[i], opy[i]);

  // 2. Fill solid white
  fillPoly(opx, opy, SOLE_N, true);

  // 3. Carve inner hollow
  int ipx[SOLE_N], ipy[SOLE_N];
  for (int i = 0; i < SOLE_N; i++) {
    float lx = SOLE_LX[i], ly = SOLE_LY[i];
    float mag = sqrt(lx*lx + ly*ly);
    if (mag < 1) mag = 1;
    footXF(lx - lx/mag*3.2f, ly - ly/mag*3.2f,
           angle, fbScale, ipx[i], ipy[i]);
  }
  fillPoly(ipx, ipy, SOLE_N, false);

  // 4. Crisp outer outline
  for (int i = 0; i < SOLE_N; i++) {
    int j = (i+1) % SOLE_N;
    oledLine(opx[i], opy[i], opx[j], opy[j], true);
  }

  // 5. Toes — filled circles (big toe largest)
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

  // 6. Heel pad indent
  {
    int hx, hy;
    footXF(0, 21, angle, fbScale, hx, hy);
    display.drawCircle(hx, hy, 3, SSD1306_BLACK);
  }

  // 7. Metatarsal dots
  for (int m = 0; m < FOOT_TOE_N; m++) {
    int mx, my;
    footXF(FOOT_META_LX[m], FOOT_META_LY[m], angle, fbScale, mx, my);
    display.drawPixel(mx, my, SSD1306_BLACK);
  }
}

// ── Direction label (top-right, 5-char wide) ──────────────────
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

// ── LR (bottom) + FB (right edge) indicator bars ─────────────
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

  // ── Wake MPU6050 (exits sleep mode) ───────────────────────
  mpuWrite(0x6B, 0x00);          // PWR_MGMT_1 = 0 → wake
  mpuWrite(0x1C, 0x00);          // ACCEL_CONFIG → ±2g range
  delay(100);

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

  // ── Average 20 samples as zero-reference ──────────────────
  float sumAX = 0, sumAY = 0, sumAZ = 0;
  float ax, ay, az;
  for (int i = 0; i < 20; i++) {
    mpuRead3(0x3B, ax, ay, az);
    sumAX += ax; sumAY += ay; sumAZ += az;
    delay(50);
  }
  offsetAX = sumAX / 20.0f;
  offsetAY = sumAY / 20.0f;
  offsetAZ = sumAZ / 20.0f;

  Serial.printf("Offsets AX:%.4f AY:%.4f AZ:%.4f\n",
                offsetAX, offsetAY, offsetAZ);

  display.clearDisplay();
  display.setCursor(18, 24); display.println(F("Zero Set! Ready."));
  display.display();
  delay(1000);
}

void loop() {
  float ax, ay, az;
  mpuRead3(0x3B, ax, ay, az);

  // ── Apply zero offset ─────────────────────────────────────
  float zax = ax - offsetAX;
  float zay = ay - offsetAY;
  // For az keep raw gravity component (needed for atan2 angle math)
  float zaz = az;   // don't subtract az offset — gravity is the reference

  // ── Compute real tilt angles (degrees) ────────────────────
  // Roll  = Left/Right lean  → rotation around X axis
  // Pitch = Front/Back lean  → rotation around Y axis
  float rollDeg  = atan2(zay, zaz) * RAD_TO_DEG;   // LR
  float pitchDeg = atan2(-zax, zaz) * RAD_TO_DEG;  // FB

  // ── Normalise to −1…+1 for visual (clamp at ±MAX_TILT_DEG)
  float lrNorm = constrain(rollDeg,  -MAX_TILT_DEG, MAX_TILT_DEG) / MAX_TILT_DEG;
  float fbNorm = constrain(pitchDeg, -MAX_TILT_DEG, MAX_TILT_DEG) / MAX_TILT_DEG;

  display.clearDisplay();
  drawSole(lrNorm, fbNorm);
  drawLabel(rollDeg, pitchDeg);
  drawBars(lrNorm, fbNorm);
  display.display();

  // ── Serial debug ──────────────────────────────────────────
  Serial.printf("Roll: %6.1f°  Pitch: %6.1f°  |  AX:%.3f AY:%.3f AZ:%.3f\n",
                rollDeg, pitchDeg, zax, zay, zaz);

  delay(150);
}
