# TiltLean
GY-271  Tilt/Lean Detector with OLED SSD1306 Display - Reads XYZ on startup and sets as zero reference (offset calibration) - Detects Left/Right lean and Front/Back lean from relative changes - Displays lean direction and angle on 128x64 OLED via I2C
USED : https://github.com/mprograms/QMC5883LCompass

## How It Works

### Zero Calibration (Startup)
```
Offset = Average of 20 readings at power-on
Zeroed Value = Raw Reading − Offset
```
This arithmetic subtraction makes your startup position the "neutral zero" for all subsequent readings.

### Axis → Direction Mapping

| Axis | Direction Detected | Positive | Negative |
|------|--------------------|----------|----------|
| **Y** | Left / Right | RIGHT | LEFT |
| **X** | Front / Back | FRONT | BACK |
| **Z** | Vertical reference | — | — |

### OLED Display Layout (128×64)
```
-- LEAN DETECTOR --
─────────────────
L/R : RIGHT
  Mag: 320
F/B : FRONT
  Mag: 150
─────────────────
Z: -42
