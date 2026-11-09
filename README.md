# ESP32 INA219 Dashboard — Setup Instructions

## 1. Hardware Wiring

| Component | ESP32 Pin |
|---|---|
| **Relay module IN** | GPIO25 |
| **Relay module VCC** | 3.3 V (or 5 V if module requires) |
| **Relay module GND** | GND |
| **INA219 VCC** | 3.3 V |
| **INA219 GND** | GND |
| **INA219 SDA** | GPIO21 |
| **INA219 SCL** | GPIO22 |
| **INA219 VIN+** | Power supply positive |
| **INA219 VIN−** | Load positive |
| **Load negative** | Power supply negative |
| **BOOT button** | GPIO0 (built-in) |

> **Important:** Place INA219 in series with the load — current must flow through VIN+ → VIN−. Do **not** exceed 32 V or 2 A.

---

## 2. Install Libraries (Arduino IDE)

1. Open **Arduino IDE** → **Tools → Board → ESP32 Dev Module**
2. Go to **Sketch → Include Library → Manage Libraries**
3. Install:
   - **Adafruit INA219** (by Adafruit)
   - **Adafruit BusIO** (auto-prompts as dependency)
4. ESP32 core (WiFi, WebServer, DNSServer, Preferences) is **bundled** — no install needed.

---

## 3. Upload the Sketch

1. Paste the full code into a new `.ino` file.
2. Select the correct **COM port**.
3. Click **Upload**.
4. Wait for `Done uploading.`

---

## 4. First Boot

1. Power the ESP32.
2. On your phone/laptop, open WiFi settings.
3. Connect to:
   - **SSID:** `ESP32_INA219`
   - **Password:** `12345678`
4. A **captive portal** may pop up automatically — if not, open a browser and go to:
   ```
   http://192.168.4.1
   ```

---

## 5. Using the Dashboard

### Dashboard tab
- Shows live **Voltage**, **Current**, **Power** updated every second.
- Shows **Relay** ON/OFF status.

### Relay tab
1. **Enable auto thresholds** → turn ON.
2. **Active HIGH relay** → turn ON for active-high relay modules, OFF for active-low.
3. Set thresholds:
   - **Voltage (V)** — e.g. `12.5`
   - **Current (A)** — e.g. `0.5`
   - **Power (W)** — e.g. `10`
4. Enable each with its **Use** toggle.
5. Tap **Save Settings**.

> Relay turns **ON** when **any** enabled threshold is met (OR logic). All settings are saved to flash automatically.

### WiFi AP tab
- Read-only info: SSID, IP, connected clients, captive portal status.
- AP is **always active** — self-heals if it drops.

### System tab
- **Uptime**, **Free heap**, **Min free heap**, **Reset reason**.
- **Restart** — reboots ESP32 (settings preserved).
- **Factory Reset** — erases all settings and reboots.

---

## 6. Factory Reset

**Option A — Button:** Hold the **BOOT** button (GPIO0) for **5 seconds**. Relay will blink 6 times, then the board reboots with default settings.

**Option B — Web:** Go to **System tab → Factory Reset → confirm**.

---

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| No AP `ESP32_INA219` | Check power; press EN/reset button once |
| Readings show `0.00` | Check I2C wiring (SDA=21, SCL=22); check INA219 has power |
| Relay never toggles | Confirm **Enable auto thresholds** is ON and threshold value > 0 and its **Use** toggle is ON |
| Relay behaves inverted | Toggle **Active HIGH relay** setting |
| Portal doesn't open | Manually browse `http://192.168.4.1` |
| Wrong readings | Verify load is wired through INA219 (VIN+ → VIN−) |
| Can't connect after factory reset | Reconnect to `ESP32_INA219` / `12345678` |

---

## 8. Quick Reference

| Item | Value |
|---|---|
| AP SSID | `ESP32_INA219` |
| AP Password | `12345678` |
| AP IP | `192.168.4.1` |
| Relay pin | GPIO25 |
| Reset button | GPIO0 (hold 5 s) |
| I2C pins | SDA=21, SCL=22 |
| INA219 I2C address | `0x40` |
| Sample rate | 1 Hz |
| NVS flush interval | 60 s (forced on Save) |

---

## 9. Safety Limits

- **Max voltage:** 32 V
- **Max current:** 2 A
- **Max clients:** 4 simultaneous
- Do **not** exceed these — sensor calibration clamps readings.
