# NLFRC Line Follower — Setup & Tuning Guide

## 1. Required Libraries
Install via Arduino IDE → Library Manager:
- `QTRSensors` by Pololu  (search: "QTR Sensors Pololu")

Board: `ESP32 Dev Module` (via Boards Manager → esp32 by Espressif)

---

## 2. Pin Assignment  (fill in YOUR wiring)

### QTR-8A Sensor Array
```
QTR Pin   →  ESP32 Pin    Notes
-----------------------------------------------
S0 (left) →  [  ]         ADC1 pins only!
S1        →  [  ]         Safe ADC1 pins:
S2        →  [  ]         32, 33, 34, 35,
S3        →  [  ]         36, 39, 25, 26
S4        →  [  ]
S5        →  [  ]
S6        →  [  ]
S7 (rght) →  [  ]
LEDON     →  [  ]         or wire to 3.3V always-on
VCC       →  3.3V
GND       →  GND
```
> ⚠️ Do NOT use ADC2 pins (0,2,4,12,13,14,15,25,26,27) — they conflict with WiFi/BT radio.

### TB6612FNG Motor Driver
```
Driver Pin  →  ESP32 Pin    Notes
-----------------------------------------------
AIN1        →  [  ]         Left motor direction
AIN2        →  [  ]         Left motor direction
PWMA        →  [  ]         Left motor speed (PWM)
BIN1        →  [  ]         Right motor direction
BIN2        →  [  ]         Right motor direction
PWMB        →  [  ]         Right motor speed (PWM)
STBY        →  [  ]         Pull HIGH to enable driver
VM          →  Battery+     Motor supply (3–13V)
VCC         →  3.3V         Logic supply
GND         →  GND
AO1, AO2    →  Left motor
BO1, BO2    →  Right motor
```

### Start Button
```
One pin → START_BUTTON_PIN → other pin → GND
(Internal pull-up enabled in code)
```

---

## 3. Wiring Diagram (ASCII)

```
          ┌───────────────────────────────────────────────┐
          │                  ESP32                        │
          │                                               │
          │  3.3V ──┬──── QTR VCC                        │
          │         └──── TB6612 VCC                      │
          │  GND  ──┬──── QTR GND                        │
          │         ├──── TB6612 GND                      │
          │         └──── Button GND                      │
          │                                               │
          │  ADC1 pins ─── QTR S0…S7                     │
          │  GPIO  ──────── QTR LEDON                     │
          │                                               │
          │  GPIO ──── AIN1 ─┐                            │
          │  GPIO ──── AIN2  │ TB6612FNG                  │
          │  GPIO ──── PWMA  │                            │
          │  GPIO ──── BIN1  │      AO1/AO2 ── Left Mtr  │
          │  GPIO ──── BIN2  │      BO1/BO2 ── Right Mtr │
          │  GPIO ──── PWMB  │                            │
          │  GPIO ──── STBY ─┘                            │
          │                                               │
          │  Battery+ ─────── TB6612 VM                  │
          └───────────────────────────────────────────────┘
```

---

## 4. Step-by-Step Startup

1. Wire everything, then open Serial Monitor at **115200 baud**
2. Upload code
3. Press boot button → **calibration phase starts**
4. Slowly sweep the robot over the black line (all 8 sensors) for ~5 seconds
5. Press button again → **robot starts running**

---

## 5. PID Tuning Order (competition method)

### Phase 1 — Get it moving
Set `BASE_SPEED = 80`, all KI = 0, all KD = 0.
Increase KP_NORMAL until the robot oscillates, then back off 30%.

### Phase 2 — Add derivative
Increase KD_NORMAL until oscillations dampen cleanly.
Good starting range: `KD = 10–30 × KP`

### Phase 3 — Speed up
Raise `BASE_SPEED` in steps of 10. Re-tune KD if oscillations return.

### Phase 4 — Sharp sections (zigzags/chevrons)
These sections need `KP_SHARP` ≈ 1.5–2× KP_NORMAL.
Adjust `SHARP_THRESHOLD` to where you want the switchover.

### Phase 5 — Integral (optional)
Small KI (0.0001–0.001) corrects steady-state drift on long straights.
Watch for windup on corners — the anti-windup clamp handles this.

---

## 6. Track Section Notes (NLFRC-specific)

| Section           | Key Challenge                  | What Helps                    |
|-------------------|--------------------------------|-------------------------------|
| Start hairpin     | Tight U-turn                   | KD high, slow entry           |
| Zigzag descent    | Rapid alternating error        | KP_SHARP, fast response       |
| Chevron arrows    | False outer path at tips       | Narrow track width helps      |
| Staircase steps   | Sharp 90° corners              | SHARP_THRESHOLD catch         |
| Diamond crossover | Intersection — sensors scatter | Straight-line momentum helps  |
| Stop circle       | All-white gap in line          | STOP_GAP_MS detection         |
| Finish arch       | All-black bar                  | FINISH_BAR_MS detection       |

---

## 7. Sensor Value Interpretation

```
Position = 0     → line under S0 (far left)
Position = 3500  → line centered
Position = 7000  → line under S7 (far right)

Error = Position - 3500
Error < 0  → robot drifted RIGHT, needs left correction
Error > 0  → robot drifted LEFT, needs right correction
```

---

## 8. Troubleshooting

| Symptom                         | Fix                                                  |
|---------------------------------|------------------------------------------------------|
| Robot spins in place            | Swap AO1/AO2 or BIN1/BIN2 on one motor              |
| Robot veers always one way      | Re-calibrate sensors; check physical alignment       |
| Oscillates badly                | Reduce KP, increase KD                              |
| Misses sharp turns              | Increase KP_SHARP, lower SHARP_THRESHOLD            |
| Stops at stop marker too often  | Increase STOP_GAP_MS                                |
| Doesn't stop at finish          | Decrease FINISH_BAR_MS                              |
| ADC readings erratic            | Ensure using ADC1 pins only                         |
| Motors don't move               | Check STBY pin is HIGH, check TB6612 VM voltage      |
