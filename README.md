# TiltLean
GY-271  Tilt/Lean Detector with OLED SSD1306 Display - Reads XYZ on startup and sets as zero reference (offset calibration) - Detects Left/Right lean and Front/Back lean from relative changes - Displays lean direction and angle on 128x64 OLED via I2C
USED : https://github.com/mprograms/QMC5883LCompass
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Required Libraries
Install these via Arduino Library Manager:

QMC5883LCompass by MPrograms
Adafruit SSD1306
Adafruit GFX Library

Wiring (I2C — shared bus)
|GY-271 / SSD1306      |  Arduino| Esp32 |
|------|--------------------|-------|
|VCC                   | 3.3V or 5V| 3.3V |
|GND                   |  GND| GND| 
|SDA                  |   A4| D21 or GPIO21 |
|SCL             |        A5|D22 or GPIO22 |

Tip: Adjust LEAN_THRESHOLD (currently 50) higher to reduce false triggers from noise, or lower it for more sensitivity.

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
```

### OUTPUT
Zeroed -> X: -2126 Y: -801 Z: -1307  |  LR: LEFT (801)  FB: BACK (2126)
Zeroed -> X: -2128 Y: -801 Z: -1326  |  LR: LEFT (801)  FB: BACK (2128)
Zeroed -> X: -2136 Y: -814 Z: -1321  |  LR: LEFT (814)  FB: BACK (2136)
Zeroed -> X: -2133 Y: -799 Z: -1316  |  LR: LEFT (799)  FB: BACK (2133)
Zeroed -> X: -2138 Y: -801 Z: -1304  |  LR: LEFT (801)  FB: BACK (2138
