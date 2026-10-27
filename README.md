# Waveshare ESP32-S3-Touch-AMOLED-2.06 Smartwatch Firmware & Companion Guide

Custom LVGL 9.5 + BLE UART smartwatch firmware ([`waveshare-watch.ino`](./waveshare-watch.ino)) and Web Bluetooth companion dashboard ([`watch-manager.html`](./watch-manager.html)) for the **[Waveshare ESP32-S3-Touch-AMOLED-2.06](https://www.waveshare.com/esp32-s3-touch-amoled-2.06.htm)**.

---

## 1. Hardware & Safe-Area Architecture

### Target Hardware
- **MCU:** ESP32-S3 (Dual-Core XTensa LX7 @ 240 MHz, 8 MB Octal PSRAM `AP_3v3`, 16 MB Flash)
- **Display:** 2.06" CO5300 QSPI AMOLED (`410 × 502` pixels, ~72 px corner radius "squircle" bezel)
- **Touch Controller:** FT3168 Capacitive Touch over I2C (`SDA=15`, `SCL=14`, `RST=9`, `INT=38`)
- **PMU:** AXP2101 Power Management IC (`0x34`)
- **RTC:** PCF85063 Real-Time Clock
- **Audio:** ES8311 I2S Codec + GPIO 46 Speaker PA

### 2.06" Rounded-Bezel Safe-Area Constants
Because the 410×502 AMOLED glass has ~72 px radius rounded corners, all UI containers are constrained within safe inset bounds defined at the top of [`waveshare-watch.ino`](./waveshare-watch.ino):

| Constant | Value | Purpose |
| :--- | :--- | :--- |
| `SAFE_TOP_Y` | `18 px` | Clears the top bezel corner curve (`chord width ≈ 356 px` at `y = 18`) |
| `SAFE_BAR_W × SAFE_BAR_H` | `336 × 40 px` | Top status & navigation pill bar |
| `SAFE_PAGE_Y` | `62 px` | Vertical start of all 4 main pages |
| `SAFE_PAGE_W × SAFE_PAGE_H` | `352 × 410 px` | Main page bounds (`y = 62..472`) |
| `SAFE_CARD_W` | `336 px` | Standard width for cards inside pages |
| `SAFE_FACE_W × SAFE_FACE_H` | `342 × 358 px` | Watch face canvas container (`0..6`) |
| `SAFE_MODAL_W × SAFE_MODAL_H` | `340 × 416 px` | Popups, keyboards, and gallery modals |

---

## 2. Compiling & Uploading (`arduino-cli`)

### Compile
```bash
/usr/local/bin/arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=huge_app,PSRAM=opi \
  /home/dannybwoi/Desktop/arduino_projects/waveshare-watch
```

### Compile & Upload to Watch (`/dev/ttyACM0`)
```bash
/usr/local/bin/arduino-cli compile -u -p /dev/ttyACM0 \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=huge_app,PSRAM=opi \
  /home/dannybwoi/Desktop/arduino_projects/waveshare-watch
```

---

## 3. Computational Boundaries & Stability Safeguards

To prevent freezes, stack overflows, or BLE link drops (`GATT status 147`), [`waveshare-watch.ino`](./waveshare-watch.ino) enforces these architectural boundaries:

1. **24 KB Main Loop Stack (`SET_LOOP_TASK_STACK_SIZE(24 * 1024)`):** Triples the default 8 KB Arduino `loopTask` stack so LVGL 9.5 flex/grid rendering, touch callbacks, and BLE operations never overflow the stack.
2. **Enforced Memory & Timing Floors:**
   - **Internal SRAM Free Floor:** `>= 40 KB` (`SAFE_MIN_INTERNAL_SRAM_BYTES`)
   - **Largest Contiguous Internal DMA Block:** `>= 24 KB` (`SAFE_MIN_LARGEST_BLOCK_BYTES`)
   - **Free `loopTask` Stack High-Water Mark:** `>= 1,536 bytes` (`SAFE_MIN_STACK_FREE_BYTES`)
   - **Loop Frame Time Budget:** `<= 40 ms` (`SAFE_MAX_LOOP_TIME_MS`)
3. **BLE Connection Stability for Google Pixel / Android:**
   - **"Just Works" BLE Security (`BLESecurity` with `ESP_LE_AUTH_BOND` + `ESP_IO_CAP_NONE`):** Completes Android's 30-second `BTM_SEC_TIMEOUT_VALUE` SMP security check immediately so the phone does not drop the link after 30 seconds.
   - **Standard BLE Battery Service (`0x180F`):** Exposed alongside the Nordic UART Service (`6E400001-...`) with a 15-second battery level keep-alive update.
   - **Deferred GATT Handshake & Core-1 Isolation:** Waits `1.8s` after `onConnect` before sending the initial `REQ:SYNC` notification and handles advertising restarts exclusively on Core 1.
   - **NVS Flash Write Guarding:** Avoids repetitive `prefs.putInt()` SPI flash writes during `TIME:`, `FACE:`, and `BRIGHT:` updates so flash cache suspension never stalls the BLE radio.
   - **Pre-Allocated Single-Label BLE Log (`bt_log_label`):** Avoids dynamic `lv_list_add_button()` and circular-scrolling label timer allocations when opening the Bluetooth page.

### Viewing Boundaries on the Watch
- Tap the **Settings (Gear)** icon on the top bar → view the **`HW BOUNDARIES`** card at the top, or tap **"Run Boundary & Heap Self-Test"** to stress-test a `16 KB` internal DMA block + `64 KB` PSRAM block on demand.
- Or send `DIAG` over Bluetooth to receive:
  ```text
  BOUNDARIES IntSRAM:172KB(blk:110KB,min:158KB) PSRAM:8120KB Stack:19800B MaxLoop:14ms Fails:0
  ```

---

## 4. Chrome Web Bluetooth Companion App ([`watch-manager.html`](./watch-manager.html))

[`watch-manager.html`](./watch-manager.html) is a single-file Web Bluetooth dashboard that runs in **Chrome on your Google Pixel** or **Chrome/Brave/Edge on your Linux PC**.

### Features
- **Auto-Syncs Pixel Clock & Live Weather:** Automatically sends `TIME:YYYY-MM-DD HH:MM:SS` on connect and whenever you tap **Sync BT** on the watch (`REQ:SYNC`), and automatically fetches live local weather via Open-Meteo (`WEATHER:...`) whenever you tap **Refresh** on the watch's Weather face (`REQ:WEATHER`).
- **One-Tap Popups & Interactive Prompts:**
  - **Send Alert Popup (`NOTIFY:Title|Body`)**
  - **Ask Accept / Reject (`REQ:Question?`)** — shows a live toast on your phone when you tap **Accept** or **Reject** on the watch.
  - **Simulate Call (`CALL:Caller Name`)** — shows when you tap **Answer** or **Decline** on the watch.
- **Remote Controls:** Switch all **7 Watch Faces** (`FACE:0`–`FACE:6`), switch **Menu Pages** (`PAGE:0`–`PAGE:3`), adjust **Brightness (`10%–100%`)**, set/disable **Alarms**, start **1m / 5m Timers**, or **Ping Watch (`FIND`)**.
- **Live Hardware Boundaries Panel:** Parses `STATUS` and `DIAG` responses into live SRAM, PSRAM, Stack, and Battery cards.

### How to Run on Your Google Pixel Over Wi-Fi
1. In this project folder on your PC, start a local HTTP server:
   ```bash
   python3 -m http.server 8080
   ```
2. Find your PC's LAN IP (`hostname -I`, e.g., `http://172.20.2.71:8080`).
3. On your **Google Pixel**, open Chrome and navigate to:
   ```text
   chrome://flags/#unsafely-treat-insecure-origin-as-secure
   ```
4. Add `http://172.20.2.71:8080` to the text box, set the dropdown to **Enabled**, and tap **Relaunch**.
5. Open **`http://172.20.2.71:8080/watch-manager.html`** in Chrome on your Pixel.
   - *(Optional)* Tap **⋮ → Add to Home screen** to install it as a standalone Pixel app.
   - *(Note)* Disconnect **Serial Bluetooth Terminal** before connecting in Chrome Web Bluetooth, as the watch connects to one active central at a time.

---

## 5. Android Apps & Background Automation on Google Pixel

### Option A: **Serial Bluetooth Terminal** (Manual Terminal & Macro Buttons)
1. Install **Serial Bluetooth Terminal** (by *Kai Morich*) from Google Play.
2. Tap **☰ → Devices → Bluetooth LE → SCAN** → select **`Waveshare-S3-Watch`**.
3. Enable **☰ → Settings → Misc → Keep screen on / Run in background** so Android doesn't suspend the connection.
4. Program the **Macro buttons (`M1`–`M7`)**:
   - **M1 (`Status`):** `STATUS`
   - **M2 (`Boundaries`):** `DIAG`
   - **M3 (`Next Face`):** `FACE:NEXT`
   - **M4 (`Find Watch`):** `FIND`
   - **M5 (`Test Call`):** `CALL:Google Pixel`
   - **M6 (`Ask Watch`):** `REQ:Ready to head out?`
   - **M7 (`Weather`):** `WEATHER:22.5,Sunny,San Francisco,55,12`

### Option B: **Automate (by LlamaLab)** (Background Notification & Call Forwarding)
Google Pixel's built-in *System Rules* and *Google Home Routines* cannot write raw BLE GATT bytes. To automatically forward real Pixel notifications or calls in the background:
1. Install **Automate** (by *LlamaLab*, free on Google Play).
2. Create a flow with:
   - **Block 1:** `Notification posted?` (or `Incoming call?`)
   - **Block 2:** `Bluetooth GATT write`
     - **Device:** `Waveshare-S3-Watch` (`1C:DB:D4:7B:71:08`)
     - **Service UUID:** `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
     - **Characteristic UUID:** `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
     - **Value:** `"NOTIFY:" ++ title ++ "|" ++ message ++ "\n"`

---

## 6. Complete BLE UART Command Reference

- **Device Name:** `Waveshare-S3-Watch`
- **Nordic UART Service UUID:** `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- **RX Characteristic (Phone → Watch Write):** `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- **TX Characteristic (Watch → Phone Notify):** `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

| Command (Phone → Watch) | Example | Description |
| :--- | :--- | :--- |
| `TIME:<YYYY-MM-DD HH:MM:SS>` | `TIME:2026-10-27 12:45:00` | Sets hardware PCF85063 RTC & system clock (also accepts unix epoch `TIME:1793130300`) |
| `NOTIFY:<Title>\|<Message>` | `NOTIFY:Pixel\|Meeting in 5m` | Wakes screen, chimes, and opens an alert modal |
| `REQ:<Question>` | `REQ:Approve login?` | Opens an **Accept / Reject** modal; replies `RESP:ACCEPTED:...` or `RESP:REJECTED:...` |
| `CALL:<Caller>` | `CALL:Mom` | Opens an **Answer / Decline** call modal; replies `CALL:ANSWERED` or `CALL:DECLINED` |
| `WEATHER:<c>,<cond>,<city>[,<hum>,<wind>]` | `WEATHER:21.5,Clear,SF,58,14` | Updates Watch Face 5 (Weather Station) |
| `STEPS:<count>` / `HR:<bpm>` | `STEPS:9500` / `HR:76` | Updates Watch Face 6 (AMOLED Sport Arc) fitness stats |
| `FACE:<0..6>` or `FACE:NEXT` | `FACE:6` | Switches active watch face (`0`=Modern, `1`=Sundial, `2`=Neon, `3`=Analog, `4`=ASCII, `5`=Weather, `6`=Sport Arc) |
| `PAGE:<0..3>` | `PAGE:2` | Switches watch page (`0`=Clock, `1`=Tools, `2`=Bluetooth, `3`=Settings) |
| `BRIGHT:<10..100>` | `BRIGHT:85` | Sets AMOLED hardware brightness percentage |
| `ALARM:<HH:MM>` / `ON` / `OFF` | `ALARM:07:30` | Sets or toggles the watch alarm |
| `TIMER:<seconds>` / `STOP` / `RESET` | `TIMER:120` | Starts, stops, or resets the countdown timer |
| `FIND` / `RING` / `BEEP` | `FIND` | Wakes screen, plays 3-tone chime, and shows "FIND MY WATCH" popup |
| `STATUS` / `PING` / `INFO` | `STATUS` | Returns `STATUS YYYY-MM-DD HH:MM:SS BATT:xx%(xxxxmV) FACE:x STEPS:xxxx` |
| `DIAG` / `BOUNDARIES` | `DIAG` | Returns live Internal SRAM, contiguous block, PSRAM, stack margin, and max loop time |
| `HELP` | `HELP` | Lists supported BLE commands |

---

## 7. Python CLI Script Example (`bleak`)

You can also send commands from any computer or Raspberry Pi using Python (`pip install bleak`):

```python
import asyncio
import sys
from datetime import datetime
from bleak import BleakClient, BleakScanner

WATCH_NAME = "Waveshare-S3-Watch"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

async def main(command: str):
    device = await BleakScanner.find_device_by_name(WATCH_NAME, timeout=10.0)
    if not device:
        print("Watch not found!")
        return

    async with BleakClient(device) as client:
        def on_notify(_, data: bytearray):
            print("Watch reply:", data.decode(errors="ignore").strip())

        await client.start_notify(TX_UUID, on_notify)
        await client.write_gatt_char(RX_UUID, (command + "\n").encode(), response=False)
        await asyncio.sleep(1.5)

if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else f"TIME:{datetime.now():%Y-%m-%d %H:%M:%S}"
    asyncio.run(main(cmd))
```
