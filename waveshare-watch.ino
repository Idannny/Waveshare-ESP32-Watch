#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <NetworkClient.h>
#include <HTTPClient.h>
#include <time.h>
#include <sys/time.h>
#include <Preferences.h>
#include <lvgl.h>
#include <math.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

#include "pin_config.h"
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "lv_conf.h"
#include "SensorPCF85063.hpp"
#include "XPowersLib.h"
#include "HWCDC.h"

#include "ESP_I2S.h"
#include "es8311.h"
#include <esp_heap_caps.h>

// Give Arduino loopTask a 24 KB stack (default is only 8 KB) so LVGL 9.5 rendering + BLE never overflow stack
SET_LOOP_TASK_STACK_SIZE(24 * 1024);

HWCDC USBSerial;

// --- Safe-Area Geometry for Waveshare 2.06" Rounded-Bezel AMOLED (410x502) ---
// The physical watch glass has ~72px radius corners ("squircle").
// Keeping UI elements within these inset coordinates prevents corner/edge clipping.
#define SAFE_TOP_Y     18
#define SAFE_BAR_W     336
#define SAFE_BAR_H     40
#define SAFE_PAGE_Y    62
#define SAFE_PAGE_W    352
#define SAFE_PAGE_H    410
#define SAFE_CARD_W    336
#define SAFE_FACE_W    342
#define SAFE_FACE_H    358
#define SAFE_MODAL_W   340
#define SAFE_MODAL_H   416

// --- High-Performance Display Buffer Config (Internal DMA SRAM) ---
#define RENDER_BUF_HEIGHT 40

uint32_t screenWidth;
uint32_t screenHeight;
lv_display_t *disp;
lv_color_t *disp_draw_buf1;
lv_color_t *disp_draw_buf2;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RESET,
                                        0 /* rotation */, LCD_WIDTH, LCD_HEIGHT,
                                        22 /* col_offset1 */, 0 /* row_offset1 */,
                                        0 /* col_offset2 */, 0 /* row_offset2 */);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);

std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       TP_RESET, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}

// --- Peripherals & Objects ---
SensorPCF85063 rtc;
XPowersPMU power;
Preferences prefs;
I2SClass i2s;
es8311_handle_t es_handle = NULL;

#define AUDIO_PA_PIN 46
#define SAMPLE_RATE 16000
bool audio_available = false;
bool audio_muted = false;
int display_brightness = 208; // Default ~80% (0-255 scale)

// --- Watch Face Options ---
const char* face_names[] = {
  "0: Modern Digital",
  "1: Sundial Simulator",
  "2: Neon Cyberpunk",
  "3: Analog Classic",
  "4: ASCII Matrix Terminal",
  "5: Weather Station",
  "6: AMOLED Sport Arc"
};
const int num_faces = 7;
int selected_face_idx = 0;

// --- Timezone Options ---
struct TimezoneOption {
  const char* name;
  const char* posix_str;
};

const TimezoneOption tz_options[] = {
  {"US Eastern (EST/EDT)", "EST5EDT,M3.2.0,M11.1.0"},
  {"US Central (CST/CDT)", "CST6CDT,M3.2.0,M11.1.0"},
  {"US Mountain (MST/MDT)", "MST7MDT,M3.2.0,M11.1.0"},
  {"US Pacific (PST/PDT)", "PST8PDT,M3.2.0,M11.1.0"},
  {"UTC", "UTC0"}
};
const int num_tz_options = 5;
int current_tz_idx = 0;

const char* ntp_server1 = "pool.ntp.org";
const char* ntp_server2 = "time.nist.gov";
const char* ntp_server3 = "time.google.com";

// --- State Variables ---
bool use_12h_format = true;
bool wifi_power_on = false; // Wi-Fi disabled (Bluetooth-only mode)
bool wifi_connect_in_progress = false;
uint32_t wifi_connect_start_ms = 0;
bool is_screen_sleeping = false;
bool wifi_scan_in_progress = false;
bool ntp_sync_in_progress = false;
uint32_t ntp_sync_start_ms = 0;
const int NUM_PAGES = 4;
int active_page_idx = 0; // 0=Clock, 1=Tools, 2=Bluetooth, 3=Settings

uint32_t last_touch_time = 0;
uint32_t auto_sleep_timeout_ms = 60000; // 60s auto-sleep (extended on touch & BLE activity)

// --- Bluetooth (BLE UART & Android Request Handler) State ---
#define BLE_DEVICE_NAME        "Waveshare-S3-Watch"
#define BLE_UART_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UART_CHAR_RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UART_CHAR_TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_BATT_SERVICE_UUID  (uint16_t)0x180F
#define BLE_BATT_CHAR_UUID     (uint16_t)0x2A19

BLEServer *ble_server = NULL;
BLECharacteristic *ble_tx_char = NULL;
BLECharacteristic *ble_rx_char = NULL;
BLECharacteristic *ble_batt_char = NULL;
BLE2902 *ble_tx_cccd = NULL;
uint32_t last_ble_keepalive_ms = 0;
bool ble_power_on = true;
bool ble_initialized = false;
volatile bool ble_device_connected = false;
bool ble_old_connected = false;
volatile uint32_t ble_connect_event_ms = 0;
volatile uint32_t ble_disconnect_event_ms = 0;
bool ble_sync_pending = false;
bool ble_manual_sync_pending = false;
bool ble_adv_restart_pending = false;
bool bt_log_dirty = true;
uint32_t last_ble_notify_ms = 0;
int last_ble_ui_state = -1;
int last_ble_ui_msg_count = -1;
char last_android_req_str[64] = "Waiting for Android...";

#define MAX_BT_MESSAGES 12
struct BtMessage {
  char text[128];
  char time_str[16];
  bool is_incoming;
};
BtMessage bt_messages[MAX_BT_MESSAGES];
int bt_msg_count = 0;

// Thread-safe single-producer/single-consumer ring buffer for BLE RX
portMUX_TYPE ble_rx_mux = portMUX_INITIALIZER_UNLOCKED;
#define BLE_RX_QUEUE_SIZE 8
char ble_rx_queue[BLE_RX_QUEUE_SIZE][192];
volatile int ble_rx_head = 0;
volatile int ble_rx_tail = 0;

// --- Computational & Memory Boundary Telemetry State ---
#define SAFE_MIN_INTERNAL_SRAM_BYTES   40960U  // 40 KB safe floor for Bluedroid + QSPI DMA
#define SAFE_MIN_LARGEST_BLOCK_BYTES   24576U  // 24 KB contiguous internal block floor
#define SAFE_MIN_STACK_FREE_BYTES      1536U   // 1.5 KB minimum free loopTask stack
#define SAFE_MAX_LOOP_TIME_MS          40U     // 40 ms frame budget threshold

volatile uint32_t boundary_alloc_fail_count = 0;
volatile size_t boundary_last_fail_size = 0;
volatile uint32_t boundary_last_fail_caps = 0;
uint32_t boundary_max_loop_ms = 0;
uint32_t boundary_last_loop_ms = 0;
uint32_t boundary_last_ui_ms = 0;
uint32_t boundary_last_serial_ms = 0;
lv_obj_t *boundary_info_lbl = NULL;
lv_obj_t *boundary_status_badge = NULL;

// --- Sport / Fitness State (Syncable via Android BLE) ---
uint32_t watch_steps_count = 8432;
int watch_heart_rate_bpm = 72;

// --- Stopwatch State ---
bool sw_running = false;
uint32_t sw_start_time = 0;
uint32_t sw_accumulated_ms = 0;
int sw_lap_count = 0;

// --- Timer State ---
bool cd_running = false;
uint32_t cd_duration_ms = 60000;
uint32_t cd_start_time = 0;
uint32_t cd_remaining_ms = 60000;

// --- Alarm State ---
bool alarm_enabled = false;
int alarm_hour = 7;
int alarm_min = 0;
bool alarm_triggered_today = false;

// --- Sundial Simulation State ---
bool sundial_manual_mode = false;
int sundial_sim_min = 720; // 12:00 PM in minutes from midnight (0..1439)

// --- Global UI Containers ---
lv_obj_t *top_bar;
lv_obj_t *bar_time_lbl;
lv_obj_t *bar_wifi_lbl = NULL;
lv_obj_t *bar_bt_lbl;
lv_obj_t *bar_batt_lbl;
lv_obj_t *bar_btn_clock;
lv_obj_t *bar_btn_tools;
lv_obj_t *bar_btn_bt;
lv_obj_t *bar_btn_wifi = NULL;
lv_obj_t *bar_btn_settings;

lv_obj_t *page_clock;
lv_obj_t *page_tools;
lv_obj_t *page_bt;
lv_obj_t *page_wifi = NULL;
lv_obj_t *page_settings;

// Watch Face View Containers (Inside page_clock)
lv_obj_t *face_container_modern;
lv_obj_t *face_container_sundial;
lv_obj_t *face_container_neon;
lv_obj_t *face_container_analog;
lv_obj_t *face_container_ascii;
lv_obj_t *face_container_weather;
lv_obj_t *face_container_amoled_arc = NULL;

// --- Weather Station State ---
struct WeatherData {
  float temp_c = 0.0;
  float temp_f = 32.0;
  int humidity = 0;
  float wind_speed = 0.0;
  int weather_code = 0;
  char condition_str[32] = "No Data";
  char city_str[32] = "Auto Location";
  float temp_max_c = 0.0;
  float temp_min_c = 0.0;
  float temp_max_f = 32.0;
  float temp_min_f = 32.0;
  bool valid = false;
  uint32_t last_update_ms = 0;
};

WeatherData weather_data;
volatile bool weather_fetch_in_progress = false;
volatile bool weather_ui_dirty = false;
char weather_status_msg[32] = "Tap to sync";
uint32_t weather_fetch_start_ms = 0;
uint32_t last_weather_fetch_time = 0;
const uint32_t WEATHER_UPDATE_INTERVAL_MS = 15 * 60 * 1000;

// Face 5 (Weather Station) Elements
lv_obj_t *weather_time_lbl;
lv_obj_t *weather_date_lbl;
lv_obj_t *weather_temp_c_lbl;
lv_obj_t *weather_temp_f_lbl;
lv_obj_t *weather_cond_lbl;
lv_obj_t *weather_details_lbl;
lv_obj_t *weather_status_lbl;
lv_obj_t *weather_btn_refresh;

// Watch Face Selector Dropdown (Top Quick Bar)
lv_obj_t *face_dropdown;

// Face 0 (Modern Digital) Elements
lv_obj_t *clock_time_label;
lv_obj_t *clock_ampm_label;
lv_obj_t *clock_date_label;
lv_obj_t *wifi_status_label;
lv_obj_t *bt_clock_status_label;
lv_obj_t *batt_status_label;
lv_obj_t *ntp_status_label;
lv_obj_t *tz_dropdown;

// Face 1 (Sundial) Elements
lv_obj_t *sundial_card;
lv_obj_t *sundial_plate;
lv_obj_t *sun_obj;
lv_obj_t *shadow_obj;
lv_obj_t *gnomon_obj;
lv_obj_t *sundial_info_lbl;
lv_obj_t *sundial_slider;
lv_obj_t *sundial_mode_btn;
lv_obj_t *sundial_mode_lbl;

// Face 2 (Neon Cyberpunk) Elements
lv_obj_t *neon_time_lbl;
lv_obj_t *neon_date_lbl;
lv_obj_t *neon_sec_bar;

// Face 3 (Analog Classic) Elements
lv_obj_t *analog_dial_bg;
lv_obj_t *analog_hour_hand;
lv_obj_t *analog_min_hand;
lv_obj_t *analog_sec_hand;

// Face 4 (ASCII Matrix Terminal) Elements
lv_obj_t *ascii_lines[5];
lv_obj_t *ascii_sub_info;

// Face 6 (AMOLED Sport Arc) Elements
lv_obj_t *arc_battery_arc = NULL;
lv_obj_t *arc_time_lbl = NULL;
lv_obj_t *arc_date_lbl = NULL;
lv_obj_t *arc_steps_lbl = NULL;
lv_obj_t *arc_hr_lbl = NULL;

// Stopwatch UI
lv_obj_t *sw_time_label;
lv_obj_t *sw_lap_lbl = NULL;
lv_obj_t *sw_btn_start;
lv_obj_t *sw_lbl_start;
lv_obj_t *sw_lap_list = NULL;

// Timer UI
lv_obj_t *cd_time_label;
lv_obj_t *cd_btn_start;
lv_obj_t *cd_lbl_start;

// Alarm UI
lv_obj_t *alarm_lbl_time;
lv_obj_t *alarm_sw_enable;

// Bluetooth UI
lv_obj_t *bt_switch = NULL;
lv_obj_t *bt_status_lbl = NULL;
lv_obj_t *bt_sub_lbl = NULL;
lv_obj_t *bt_msg_list = NULL;
lv_obj_t *bt_log_label = NULL;
lv_obj_t *bt_compose_modal = NULL;
lv_obj_t *bt_compose_ta = NULL;
lv_obj_t *bt_req_modal = NULL;
char pending_android_accept_reply[64] = "RESP:ACCEPTED";
char pending_android_reject_reply[64] = "RESP:REJECTED";

// Wi-Fi UI (Disabled / Hidden while Wi-Fi menu is removed)
lv_obj_t *wifi_hdr = NULL;
lv_obj_t *lbl_sw = NULL;
lv_obj_t *lbl_scan = NULL;
lv_obj_t *wifi_switch = NULL;
lv_obj_t *wifi_list = NULL;
lv_obj_t *btn_scan = NULL;
lv_obj_t *wifi_page_status_lbl = NULL;
lv_obj_t *pwd_modal = NULL;
lv_obj_t *pwd_ta = NULL;
lv_obj_t *keyboard = NULL;
char selected_ssid[64] = "";

// Settings & Manual Time Modal UI
lv_obj_t *brightness_slider;
lv_obj_t *brightness_lbl_val;
lv_obj_t *audio_switch;
lv_obj_t *format_12h_switch = NULL;
lv_obj_t *settings_face_dropdown;
lv_obj_t *time_modal = NULL;
lv_obj_t *tm_lbl_time = NULL;
lv_obj_t *tm_lbl_date = NULL;
int edit_year = 2026, edit_month = 10, edit_day = 18;
int edit_hour = 12, edit_min = 0, edit_sec = 0;

// Touch Gestures & Watch Face Gallery UI
lv_obj_t *gallery_modal = NULL;
lv_obj_t *bar_page_dots = NULL;

uint32_t last_clock_update = 0;
uint32_t last_pmu_update = 0;
bool rtc_available = false;
bool pmu_available = false;
bool system_time_valid = false;

// Forward declarations
void create_ui();
void switch_to_page(int page_idx);
void select_watch_face(int face_idx);
void get_watch_datetime(int &year, int &month, int &day, int &hour, int &minute, int &second);
void set_watch_datetime(int year, int month, int day, int hour, int minute, int second);
void open_manual_time_modal();
void init_ble_service();
void set_ble_power(bool enable);
void update_ble_status();
void send_ble_message(const char* text, bool log_to_ui = true);
void add_bt_message_log(const char* text, bool is_incoming);
void refresh_bt_message_list();
void open_bt_compose_modal();
void open_android_request_modal(const char* title_str, const char* body_str, const char* accept_reply, const char* reject_reply);
void request_android_ble_sync();
void update_clock_display();
void update_pmu_display();
void update_tools_display();
void update_sundial_display();
void update_neon_display(int hour, int minute, int sec, int year, int month, int day);
void update_analog_display(int hour, int minute, int sec);
void update_ascii_display(int hour, int minute, int sec, int year, int month, int day);
void update_weather_display();
void update_amoled_arc_display(int hour, int minute, int sec, int year, int month, int day);
void fetch_weather();
void interpret_weather_code(int code, char* buf, size_t len);
const char* get_weather_symbol(int code);
static void weather_refresh_btn_cb(lv_event_t * e);
void render_sundial(int total_minutes);
void update_wifi_scan_results();
void update_wifi_connect_status();
void update_ntp_sync_status();
void start_wifi_scan();
void connect_to_wifi(const char* ssid, const char* password);
void set_wifi_power(bool enable);
void start_ntp_sync();
void sleep_screen();
void wake_screen();
void trigger_alert_popup(const char* title_str, const char* msg_str);
void play_beep(int freq_hz, int duration_ms);
void set_amoled_brightness(uint8_t b);
void open_watch_face_gallery();
static void screen_gesture_event_cb(lv_event_t * e);
static void watch_face_long_press_cb(lv_event_t * e);


// --- AMOLED Hardware Brightness Control ---
void set_amoled_brightness(uint8_t b) {
  if (display_brightness == (int)b) return;
  display_brightness = b;
  gfx->setBrightness(display_brightness);
}

// --- ES8311 Audio Init & Tone Beep Generator ---
esp_err_t init_audio() {
  pinMode(AUDIO_PA_PIN, OUTPUT);
  digitalWrite(AUDIO_PA_PIN, HIGH);

  i2s.setPins(41, 45, 40, 42, 16);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    USBSerial.println("I2S init failed!");
    return ESP_FAIL;
  }

  es_handle = es8311_create(0, ES8311_ADDRRES_0);
  if (!es_handle) {
    USBSerial.println("ES8311 create failed!");
    return ESP_FAIL;
  }

  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = SAMPLE_RATE * 256,
    .sample_frequency = SAMPLE_RATE
  };

  if (es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) {
    USBSerial.println("ES8311 init failed!");
    return ESP_FAIL;
  }
  es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency);
  es8311_microphone_config(es_handle, false);
  es8311_voice_volume_set(es_handle, 85, NULL);

  audio_available = true;
  USBSerial.println("ES8311 Audio Codec & Speaker PA Initialized!");
  return ESP_OK;
}

void play_beep(int freq_hz, int duration_ms) {
  if (!audio_available || audio_muted) return;
  if (duration_ms <= 0) return;
  // Keep tones short when BLE is connected so synchronous I2S writes never stall BLE supervision
  int max_dur = ble_device_connected ? 45 : 100;
  if (duration_ms > max_dur) duration_ms = max_dur;

  size_t total_samples = (SAMPLE_RATE * (size_t)duration_ms) / 1000;
  // Use a small stack chunk (192 stereo frames = 768 bytes) instead of heap malloc/free
  int16_t chunk[192 * 2];
  float phase_inc = (2.0f * (float)M_PI * (float)freq_hz) / (float)SAMPLE_RATE;
  float phase = 0.0f;

  size_t written = 0;
  while (written < total_samples) {
    size_t n = total_samples - written;
    if (n > 192) n = 192;
    for (size_t i = 0; i < n; i++) {
      int16_t sample = (int16_t)(sinf(phase) * 11000.0f);
      chunk[i * 2] = sample;
      chunk[i * 2 + 1] = sample;
      phase += phase_inc;
      if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
    }
    i2s.write((const uint8_t *)chunk, n * 2 * sizeof(int16_t));
    written += n;
  }
}

// --- Computational & Memory Boundary Monitor Helpers ---
void on_heap_alloc_failed(size_t size, uint32_t caps, const char *function_name) {
  boundary_alloc_fail_count++;
  boundary_last_fail_size = size;
  boundary_last_fail_caps = caps;
  USBSerial.printf("[BOUNDARY ALERT] Heap alloc failed! size=%u caps=0x%X fn=%s int_free=%u int_largest=%u\n",
                   (unsigned int)size,
                   (unsigned int)caps,
                   function_name ? function_name : "?",
                   (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned int)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

bool is_within_computational_boundaries(char *reason_buf, size_t reason_len) {
  size_t int_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t int_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  size_t int_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  uint32_t stack_free = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

  if (boundary_alloc_fail_count > 0) {
    if (reason_buf) snprintf(reason_buf, reason_len, "WARN: %lu alloc fail(s)", (unsigned long)boundary_alloc_fail_count);
    return false;
  }
  if (int_largest < SAFE_MIN_LARGEST_BLOCK_BYTES || int_free < SAFE_MIN_INTERNAL_SRAM_BYTES) {
    if (reason_buf) snprintf(reason_buf, reason_len, "LOW SRAM: %uKB (%uKB blk)", (unsigned int)(int_free / 1024), (unsigned int)(int_largest / 1024));
    return false;
  }
  if (stack_free < SAFE_MIN_STACK_FREE_BYTES) {
    if (reason_buf) snprintf(reason_buf, reason_len, "LOW STACK: %lu B free", (unsigned long)stack_free);
    return false;
  }
  if (int_min < 20480U) {
    if (reason_buf) snprintf(reason_buf, reason_len, "SRAM DIPPED: min %uKB", (unsigned int)(int_min / 1024));
    return false;
  }
  if (reason_buf) snprintf(reason_buf, reason_len, "OK (Safe Headroom)");
  return true;
}

void format_boundary_report(char *buf, size_t len) {
  size_t int_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t int_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  size_t int_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  uint32_t stack_free = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

  snprintf(buf, len,
           "Int SRAM: %uKB free | %uKB blk (min %uKB)\n"
           "PSRAM: %uKB free | Stack: %luB free\n"
           "Loop: %lums (max %lums) | Fails: %lu",
           (unsigned int)(int_free / 1024),
           (unsigned int)(int_largest / 1024),
           (unsigned int)(int_min / 1024),
           (unsigned int)(psram_free / 1024),
           (unsigned long)stack_free,
           (unsigned long)boundary_last_loop_ms,
           (unsigned long)boundary_max_loop_ms,
           (unsigned long)boundary_alloc_fail_count);
}

void update_computational_boundaries(uint32_t loop_elapsed_ms) {
  boundary_last_loop_ms = loop_elapsed_ms;
  if (loop_elapsed_ms > boundary_max_loop_ms && millis() > 5000) {
    boundary_max_loop_ms = loop_elapsed_ms;
  }

  uint32_t now = millis();
  if (now - boundary_last_ui_ms >= 1500) {
    boundary_last_ui_ms = now;
    if (boundary_info_lbl && !is_screen_sleeping && active_page_idx == 3) {
      char rep[192];
      format_boundary_report(rep, sizeof(rep));
      lv_label_set_text(boundary_info_lbl, rep);
    }
    if (boundary_status_badge && !is_screen_sleeping && active_page_idx == 3) {
      char reason[48];
      bool ok = is_within_computational_boundaries(reason, sizeof(reason));
      lv_label_set_text(boundary_status_badge, reason);
      lv_obj_set_style_text_color(boundary_status_badge, ok ? lv_color_hex(0xA6E3A1) : lv_color_hex(0xF38BA8), 0);
    }
  }

  if (now - boundary_last_serial_ms >= 5000) {
    boundary_last_serial_ms = now;
    char reason[48];
    bool ok = is_within_computational_boundaries(reason, sizeof(reason));
    USBSerial.printf("[BOUNDARY] %s | IntFree=%uKB IntBlk=%uKB IntMin=%uKB PSRAM=%uKB Stack=%uB Loop=%lums(max %lums) BLE=%s\n",
                     ok ? "SAFE" : reason,
                     (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                     (unsigned int)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
                     (unsigned int)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
                     (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                     (unsigned int)uxTaskGetStackHighWaterMark(NULL),
                     (unsigned long)boundary_last_loop_ms,
                     (unsigned long)boundary_max_loop_ms,
                     ble_device_connected ? "CONN" : (ble_power_on ? "ADV" : "OFF"));
  }
}

void run_boundary_self_test() {
  uint32_t t0 = millis();
  bool heap_ok = heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true);
  // Test temporary 24KB BLE/DMA headroom allocation in Internal SRAM
  void *test_blk = heap_caps_malloc(SAFE_MIN_LARGEST_BLOCK_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  bool blk_ok = (test_blk != NULL);
  if (test_blk) heap_caps_free(test_blk);
  uint32_t dt = millis() - t0;

  size_t int_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
  size_t int_blk = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024;
  size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
  uint32_t stack_free = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

  char msg[192];
  snprintf(msg, sizeof(msg),
           "Heap Integrity: %s | 24KB Probe: %s\n"
           "Int SRAM: %uKB (%uKB max blk)\n"
           "PSRAM: %uKB | Stack: %luB free\n"
           "Max Loop: %lums | Check: %lums",
           heap_ok ? "PASS" : "ERR",
           blk_ok ? "PASS" : "LOW",
           (unsigned int)int_free,
           (unsigned int)int_blk,
           (unsigned int)psram_free,
           (unsigned long)stack_free,
           (unsigned long)boundary_max_loop_ms,
           (unsigned long)dt);

  USBSerial.printf("[SELF-TEST] %s\n", msg);
  trigger_alert_popup((heap_ok && blk_ok) ? "BOUNDARIES: PASS" : "BOUNDARY WARNING", msg);
}

// --- Display Sleep / Wake Functions ---
void sleep_screen() {
  if (is_screen_sleeping) return;
  is_screen_sleeping = true;
  gfx->displayOff();
  USBSerial.println("Screen: Sleep (Display Off)");
}

void wake_screen() {
  if (!is_screen_sleeping) return;
  is_screen_sleeping = false;
  gfx->displayOn();
  last_touch_time = millis();
  USBSerial.println("Screen: Woken Up (Display On)");
}

// --- Chunked DMA Flush Callback for LVGL ---
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  lv_disp_flush_ready(disp);
}

// --- Touchpad Read Callback ---
static bool touch_was_pressed = false;
static uint32_t last_tp_poll_ms = 0;

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  uint32_t now = millis();
  bool should_read = FT3168->IIC_Interrupt_Flag ||
                     touch_was_pressed ||
                     (digitalRead(TP_INT) == LOW) ||
                     (now - last_tp_poll_ms >= 20);

  if (should_read) {
    FT3168->IIC_Interrupt_Flag = false;
    last_tp_poll_ms = now;

    int32_t finger_num = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
    if (finger_num > 0) {
      int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
      int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

      if (touchX >= 0 && touchY >= 0 && touchX < LCD_WIDTH && touchY < LCD_HEIGHT) {
        if (!touch_was_pressed) {
          USBSerial.printf("[TOUCH] Pressed at x=%ld, y=%ld\n", (long)touchX, (long)touchY);
          if (top_bar) {
            lv_obj_set_style_border_color(top_bar, lv_color_hex(0x00E5FF), 0);
            lv_obj_set_style_border_width(top_bar, 2, 0);
          }
        }
        touch_was_pressed = true;
        last_touch_time = now;

        if (is_screen_sleeping) {
          wake_screen();
          data->state = LV_INDEV_STATE_REL;
          return;
        }

        data->state = LV_INDEV_STATE_PR;
        data->point.x = touchX;
        data->point.y = touchY;
        return;
      }
    }
  }

  if (touch_was_pressed && top_bar) {
    lv_obj_set_style_border_color(top_bar, lv_color_hex(0x313244), 0);
    lv_obj_set_style_border_width(top_bar, 1, 0);
  }
  touch_was_pressed = false;
  data->state = LV_INDEV_STATE_REL;
}

void rounder_event_cb(lv_event_t *e) {
  lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
  area->x1 = (area->x1 >> 1) << 1;
  area->y1 = (area->y1 >> 1) << 1;
  area->x2 = ((area->x2 >> 1) << 1) + 1;
  area->y2 = ((area->y2 >> 1) << 1) + 1;
}

uint32_t millis_cb(void) {
  return millis();
}

// --- Trigger Alert Modal (Single-Instance Guarded) ---
static lv_obj_t *alert_modal = NULL;

void trigger_alert_popup(const char* title_str, const char* msg_str) {
  wake_screen();
  play_beep(1046, 90);

  if (alert_modal != NULL) {
    lv_obj_delete_async(alert_modal);
    alert_modal = NULL;
  }

  alert_modal = lv_obj_create(lv_scr_act());
  lv_obj_set_size(alert_modal, 336, 230);
  lv_obj_center(alert_modal);
  lv_obj_set_style_bg_color(alert_modal, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(alert_modal, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_border_width(alert_modal, 3, 0);
  lv_obj_set_style_radius(alert_modal, 24, 0);
  lv_obj_set_style_pad_all(alert_modal, 10, 0);
  lv_obj_remove_flag(alert_modal, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * t_lbl = lv_label_create(alert_modal);
  lv_label_set_text(t_lbl, title_str);
  lv_obj_set_style_text_font(t_lbl, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(t_lbl, lv_color_hex(0xF38BA8), 0);
  lv_obj_align(t_lbl, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t * m_lbl = lv_label_create(alert_modal);
  lv_label_set_text(m_lbl, msg_str);
  lv_obj_set_width(m_lbl, 304);
  lv_label_set_long_mode(m_lbl, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(m_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(m_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(m_lbl, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(m_lbl, LV_ALIGN_CENTER, 0, -4);

  lv_obj_t * btn_ok = lv_button_create(alert_modal);
  lv_obj_set_size(btn_ok, 160, 38);
  lv_obj_align(btn_ok, LV_ALIGN_BOTTOM_MID, 0, -4);
  lv_obj_set_style_bg_color(btn_ok, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_radius(btn_ok, 14, 0);
  lv_obj_t * lbl_ok = lv_label_create(btn_ok);
  lv_label_set_text(lbl_ok, "Dismiss");
  lv_obj_set_style_text_color(lbl_ok, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_ok);

  lv_obj_add_event_cb(btn_ok, [](lv_event_t * e) {
    play_beep(880, 60);
    if (alert_modal) {
      lv_obj_delete_async(alert_modal);
      alert_modal = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);
}

// --- Unified Date & Time Get/Set Helpers (RTC + System Clock) ---
static int days_in_month(int year, int month) {
  static const int dim[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12) return 30;
  if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
    return 29;
  }
  return dim[month - 1];
}

void get_watch_datetime(int &year, int &month, int &day, int &hour, int &minute, int &second) {
  if (rtc_available) {
    RTC_DateTime dt = rtc.getDateTime();
    year = dt.getYear();
    month = dt.getMonth();
    day = dt.getDay();
    hour = dt.getHour();
    minute = dt.getMinute();
    second = dt.getSecond();
    if (year >= 2024 && year <= 2099 && month >= 1 && month <= 12 && day >= 1 && day <= 31) {
      return;
    }
  }

  struct timeval tv;
  if (gettimeofday(&tv, NULL) == 0 && tv.tv_sec > 1700000000) {
    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);
    year = timeinfo.tm_year + 1900;
    month = timeinfo.tm_mon + 1;
    day = timeinfo.tm_mday;
    hour = timeinfo.tm_hour;
    minute = timeinfo.tm_min;
    second = timeinfo.tm_sec;
    return;
  }

  // Fallback using uptime from saved or default baseline
  uint32_t up_sec = millis() / 1000;
  year = 2026;
  month = 10;
  day = 16;
  hour = (12 + (up_sec / 3600)) % 24;
  minute = (up_sec / 60) % 60;
  second = up_sec % 60;
}

void set_watch_datetime(int year, int month, int day, int hour, int minute, int second) {
  if (year < 2024) year = 2024;
  if (year > 2099) year = 2099;
  if (month < 1) month = 1;
  if (month > 12) month = 12;
  int max_d = days_in_month(year, month);
  if (day < 1) day = 1;
  if (day > max_d) day = max_d;
  hour = (hour + 24) % 24;
  minute = (minute + 60) % 60;
  second = (second + 60) % 60;

  // 1. Update hardware PCF85063 RTC if present
  if (rtc_available) {
    rtc.setDateTime(year, month, day, hour, minute, second);
  }

  // 2. Update ESP32 POSIX system clock
  struct tm t = {};
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  t.tm_isdst = -1;
  time_t epoch = mktime(&t);
  struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, NULL);
  system_time_valid = true;

  // 3. Persist in Preferences ONLY if date changed (avoids 5 slow NVS flash writes stalling BLE radio)
  static int last_saved_yr = -1, last_saved_mo = -1, last_saved_dy = -1;
  if (year != last_saved_yr || month != last_saved_mo || day != last_saved_dy) {
    last_saved_yr = year;
    last_saved_mo = month;
    last_saved_dy = day;
    prefs.putInt("saved_yr", year);
    prefs.putInt("saved_mo", month);
    prefs.putInt("saved_dy", day);
    prefs.putInt("saved_hr", hour);
    prefs.putInt("saved_mn", minute);
  }

  if (ntp_status_label) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Clock Set: %02d:%02d:%02d", hour, minute, second);
    lv_label_set_text(ntp_status_label, buf);
  }

  last_clock_update = 0; // Force immediate display refresh
  update_clock_display();
}

// --- Manual Clock & Date Picker Modal ---
static void update_time_modal_labels() {
  if (tm_lbl_time) {
    char tbuf[48];
    if (use_12h_format) {
      int h12 = edit_hour % 12;
      if (h12 == 0) h12 = 12;
      snprintf(tbuf, sizeof(tbuf), "%02d : %02d : %02d %s",
               h12, edit_min, edit_sec, edit_hour >= 12 ? "PM" : "AM");
    } else {
      snprintf(tbuf, sizeof(tbuf), "%02d : %02d : %02d (24H)",
               edit_hour, edit_min, edit_sec);
    }
    lv_label_set_text(tm_lbl_time, tbuf);
  }
  if (tm_lbl_date) {
    char dbuf[48];
    snprintf(dbuf, sizeof(dbuf), "%04d - %02d - %02d", edit_year, edit_month, edit_day);
    lv_label_set_text(tm_lbl_date, dbuf);
  }
}

static void time_edit_adj_cb(lv_event_t * e) {
  play_beep(850, 30);
  intptr_t code = (intptr_t)lv_event_get_user_data(e);
  switch (code) {
    case 1:  edit_hour = (edit_hour + 1) % 24; break;
    case -1: edit_hour = (edit_hour + 23) % 24; break;
    case 2:  edit_min = (edit_min + 1) % 60; break;
    case -2: edit_min = (edit_min + 59) % 60; break;
    case 3:  edit_sec = (edit_sec + 5) % 60; break;
    case -3: edit_sec = 0; break;
    case 4:  edit_year = (edit_year < 2099) ? (edit_year + 1) : 2024; break;
    case -4: edit_year = (edit_year > 2024) ? (edit_year - 1) : 2099; break;
    case 5:  edit_month = (edit_month % 12) + 1; break;
    case -5: edit_month = ((edit_month + 10) % 12) + 1; break;
    case 6: {
      int max_d = days_in_month(edit_year, edit_month);
      edit_day = (edit_day % max_d) + 1;
      break;
    }
    case -6: {
      int max_d = days_in_month(edit_year, edit_month);
      edit_day = ((edit_day + max_d - 2) % max_d) + 1;
      break;
    }
  }
  int max_d = days_in_month(edit_year, edit_month);
  if (edit_day > max_d) edit_day = max_d;
  update_time_modal_labels();
}

void open_manual_time_modal() {
  if (time_modal != NULL) return;
  play_beep(950, 50);

  get_watch_datetime(edit_year, edit_month, edit_day, edit_hour, edit_min, edit_sec);

  time_modal = lv_obj_create(lv_scr_act());
  lv_obj_set_size(time_modal, SAFE_MODAL_W, SAFE_MODAL_H);
  lv_obj_center(time_modal);
  lv_obj_set_style_bg_color(time_modal, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(time_modal, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_border_width(time_modal, 3, 0);
  lv_obj_set_style_radius(time_modal, 26, 0);
  lv_obj_set_style_pad_all(time_modal, 8, 0);
  lv_obj_remove_flag(time_modal, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * title = lv_label_create(time_modal);
  lv_label_set_text(title, "SET TIME & DATE");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0x89B4FA), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

  // Live Time Preview Box
  lv_obj_t * preview_box = lv_obj_create(time_modal);
  lv_obj_set_size(preview_box, 314, 66);
  lv_obj_align(preview_box, LV_ALIGN_TOP_MID, 0, 24);
  lv_obj_set_style_bg_color(preview_box, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(preview_box, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_border_width(preview_box, 1, 0);
  lv_obj_set_style_radius(preview_box, 14, 0);
  lv_obj_remove_flag(preview_box, LV_OBJ_FLAG_SCROLLABLE);

  tm_lbl_time = lv_label_create(preview_box);
  lv_obj_set_style_text_font(tm_lbl_time, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(tm_lbl_time, lv_color_hex(0xA6E3A1), 0);
  lv_obj_align(tm_lbl_time, LV_ALIGN_TOP_MID, 0, -6);

  tm_lbl_date = lv_label_create(preview_box);
  lv_obj_set_style_text_font(tm_lbl_date, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(tm_lbl_date, lv_color_hex(0xF9E2AF), 0);
  lv_obj_align(tm_lbl_date, LV_ALIGN_BOTTOM_MID, 0, 4);

  update_time_modal_labels();

  // Helper lambda to create adjustment rows (sized for 340px safe width)
  auto make_adj_group = [](lv_obj_t * parent, const char* label_txt, int x_ofs, int y_ofs, intptr_t dec_code, intptr_t inc_code) {
    lv_obj_t * col = lv_obj_create(parent);
    lv_obj_set_size(col, 98, 114);
    lv_obj_align(col, LV_ALIGN_TOP_LEFT, x_ofs, y_ofs);
    lv_obj_set_style_bg_color(col, lv_color_hex(0x1E1E2E), 0);
    lv_obj_set_style_border_color(col, lv_color_hex(0x313244), 0);
    lv_obj_set_style_border_width(col, 1, 0);
    lv_obj_set_style_radius(col, 12, 0);
    lv_obj_set_style_pad_all(col, 3, 0);
    lv_obj_remove_flag(col, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * btn_inc = lv_button_create(col);
    lv_obj_set_size(btn_inc, 84, 32);
    lv_obj_align(btn_inc, LV_ALIGN_TOP_MID, 0, 2);
    lv_obj_set_style_bg_color(btn_inc, lv_color_hex(0x313244), 0);
    lv_obj_t * l_inc = lv_label_create(btn_inc);
    lv_label_set_text(l_inc, LV_SYMBOL_PLUS);
    lv_obj_center(l_inc);
    lv_obj_add_event_cb(btn_inc, time_edit_adj_cb, LV_EVENT_CLICKED, (void*)inc_code);

    lv_obj_t * lbl = lv_label_create(col);
    lv_label_set_text(lbl, label_txt);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xCDD6F4), 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * btn_dec = lv_button_create(col);
    lv_obj_set_size(btn_dec, 84, 32);
    lv_obj_align(btn_dec, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_set_style_bg_color(btn_dec, lv_color_hex(0x313244), 0);
    lv_obj_t * l_dec = lv_label_create(btn_dec);
    lv_label_set_text(l_dec, dec_code == -3 ? "00" : LV_SYMBOL_MINUS);
    lv_obj_center(l_dec);
    lv_obj_add_event_cb(btn_dec, time_edit_adj_cb, LV_EVENT_CLICKED, (void*)dec_code);
  };

  // Row 1: HOUR, MIN, SEC
  make_adj_group(time_modal, "HOUR",  4,   98, -1, 1);
  make_adj_group(time_modal, "MIN",   110, 98, -2, 2);
  make_adj_group(time_modal, "SEC",   216, 98, -3, 3);

  // Row 2: YEAR, MONTH, DAY
  make_adj_group(time_modal, "YEAR",  4,   220, -4, 4);
  make_adj_group(time_modal, "MONTH", 110, 220, -5, 5);
  make_adj_group(time_modal, "DAY",   216, 220, -6, 6);

  // Save & Cancel Buttons
  lv_obj_t * btn_save = lv_button_create(time_modal);
  lv_obj_set_size(btn_save, 148, 40);
  lv_obj_align(btn_save, LV_ALIGN_BOTTOM_LEFT, 4, -2);
  lv_obj_set_style_bg_color(btn_save, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_radius(btn_save, 16, 0);
  lv_obj_t * lbl_save = lv_label_create(btn_save);
  lv_label_set_text(lbl_save, LV_SYMBOL_OK " Save");
  lv_obj_set_style_text_color(lbl_save, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_save);

  lv_obj_add_event_cb(btn_save, [](lv_event_t * e) {
    play_beep(1200, 80);
    set_watch_datetime(edit_year, edit_month, edit_day, edit_hour, edit_min, edit_sec);
    if (time_modal) {
      lv_obj_delete_async(time_modal);
      time_modal = NULL;
      tm_lbl_time = NULL;
      tm_lbl_date = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_cancel = lv_button_create(time_modal);
  lv_obj_set_size(btn_cancel, 148, 40);
  lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_RIGHT, -4, -2);
  lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_radius(btn_cancel, 16, 0);
  lv_obj_t * lbl_cancel = lv_label_create(btn_cancel);
  lv_label_set_text(lbl_cancel, LV_SYMBOL_CLOSE " Cancel");
  lv_obj_set_style_text_color(lbl_cancel, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_cancel);

  lv_obj_add_event_cb(btn_cancel, [](lv_event_t * e) {
    play_beep(500, 50);
    if (time_modal) {
      lv_obj_delete_async(time_modal);
      time_modal = NULL;
      tm_lbl_time = NULL;
      tm_lbl_date = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);
}

// --- BLE UART Server Callbacks & Android Request System ---
// IMPORTANT: Callbacks run on Core 0 inside the Bluedroid BTC task.
// Never call BLEDevice::startAdvertising(), notify(), or LVGL functions directly inside these callbacks!
class WatchBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    ble_connect_event_ms = millis();
    ble_device_connected = true;
  }
  void onDisconnect(BLEServer *pServer) override {
    ble_disconnect_event_ms = millis();
    ble_device_connected = false;
  }
};

class WatchBleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    const uint8_t *raw = pCharacteristic->getData();
    size_t len = pCharacteristic->getLength();
    if (!raw || len == 0) return;

    // Parse lines in-place without dynamic Arduino String heap allocations on the BTC stack
    size_t start = 0;
    while (start < len) {
      size_t nl = start;
      while (nl < len && raw[nl] != '\n' && raw[nl] != '\r') {
        nl++;
      }
      size_t seg_len = nl - start;
      // Trim leading spaces
      while (seg_len > 0 && (raw[start] == ' ' || raw[start] == '\t')) {
        start++;
        seg_len--;
      }
      // Trim trailing spaces
      while (seg_len > 0 && (raw[start + seg_len - 1] == ' ' || raw[start + seg_len - 1] == '\t')) {
        seg_len--;
      }
      if (seg_len > 0) {
        if (seg_len >= sizeof(ble_rx_queue[0])) {
          seg_len = sizeof(ble_rx_queue[0]) - 1;
        }
        portENTER_CRITICAL(&ble_rx_mux);
        int next_head = (ble_rx_head + 1) % BLE_RX_QUEUE_SIZE;
        if (next_head != ble_rx_tail) {
          memcpy(ble_rx_queue[ble_rx_head], raw + start, seg_len);
          ble_rx_queue[ble_rx_head][seg_len] = '\0';
          ble_rx_head = next_head;
        }
        portEXIT_CRITICAL(&ble_rx_mux);
      }
      start = nl + 1;
    }
  }
};

void init_ble_service() {
  if (ble_initialized) return;

  USBSerial.printf("Free Internal SRAM before BLE init: %u bytes (largest block: %u)\n",
                   (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned int)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

  BLEDevice::init(BLE_DEVICE_NAME);
  if (!BLEDevice::getInitialized()) {
    USBSerial.println("ERROR: BLEDevice::init failed (HCI/NimBLE init error)");
    return;
  }
  // Use a safe 185-byte MTU instead of 517 to conserve contiguous Internal SRAM during GATT connect
  BLEDevice::setMTU(185);

  // Configure "Just Works" BLE SMP Security/Bonding so Google Pixel's 30-second
  // BTM_SEC_TIMEOUT_VALUE security check completes immediately instead of dropping the link!
  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  ble_server = BLEDevice::createServer();
  if (!ble_server) {
    USBSerial.println("ERROR: BLEDevice::createServer returned NULL");
    return;
  }
  ble_server->setCallbacks(new WatchBleServerCallbacks());

  // Standard BLE Battery Service (0x180F) so Android recognizes a standard peripheral profile
  BLEService *batt_service = ble_server->createService(BLEUUID(BLE_BATT_SERVICE_UUID));
  if (batt_service) {
    ble_batt_char = batt_service->createCharacteristic(
      BLEUUID(BLE_BATT_CHAR_UUID),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    if (ble_batt_char) {
      ble_batt_char->addDescriptor(new BLE2902());
      uint8_t init_pct = 100;
      ble_batt_char->setValue(&init_pct, 1);
    }
    batt_service->start();
  }

  BLEService *service = ble_server->createService(BLE_UART_SERVICE_UUID);
  if (!service) {
    USBSerial.println("ERROR: createService returned NULL");
    return;
  }

  ble_tx_char = service->createCharacteristic(
    BLE_UART_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_READ
  );
  if (ble_tx_char) {
    ble_tx_cccd = new BLE2902();
    ble_tx_char->addDescriptor(ble_tx_cccd);
    ble_tx_char->setValue("Waveshare-S3-Watch Ready");
  }

  ble_rx_char = service->createCharacteristic(
    BLE_UART_CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR |
    BLECharacteristic::PROPERTY_READ
  );
  if (ble_rx_char) {
    ble_rx_char->setCallbacks(new WatchBleRxCallbacks());
  }

  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  if (adv) {
    adv->addServiceUUID(BLEUUID(BLE_BATT_SERVICE_UUID));
    adv->addServiceUUID(BLE_UART_SERVICE_UUID);
    adv->setAppearance(0x00C1); // Smartwatch BLE appearance for Android
    adv->setScanResponse(true);
    // 20ms - 40ms preferred connection interval (Android's most stable window)
    adv->setMinPreferred(0x10);
    adv->setMaxPreferred(0x20);
    BLEDevice::startAdvertising();
  }

  ble_initialized = true;
  USBSerial.printf("BLE Service Initialized! Internal SRAM after BLE init: %u bytes (largest block: %u)\n",
                   (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned int)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

void set_ble_power(bool enable) {
  ble_power_on = enable;
  prefs.putBool("ble_power", ble_power_on);

  if (ble_power_on) {
    if (!ble_initialized) {
      init_ble_service();
    } else if (ble_server) {
      BLEDevice::startAdvertising();
    }
    if (bar_bt_lbl) lv_obj_set_style_text_color(bar_bt_lbl, lv_color_hex(0x89B4FA), 0);
  } else {
    if (ble_initialized && ble_server) {
      BLEDevice::stopAdvertising();
      if (ble_device_connected) {
        ble_server->disconnect(ble_server->getConnId());
      }
      ble_device_connected = false;
    }
    if (bar_bt_lbl) lv_obj_set_style_text_color(bar_bt_lbl, lv_color_hex(0x6C7086), 0);
  }
}

void add_bt_message_log(const char* text, bool is_incoming) {
  int yr, mo, dy, hr, mn, sc;
  get_watch_datetime(yr, mo, dy, hr, mn, sc);

  if (bt_msg_count >= MAX_BT_MESSAGES) {
    for (int i = 0; i < MAX_BT_MESSAGES - 1; i++) {
      bt_messages[i] = bt_messages[i + 1];
    }
    bt_msg_count = MAX_BT_MESSAGES - 1;
  }

  BtMessage &m = bt_messages[bt_msg_count++];
  strncpy(m.text, text, sizeof(m.text) - 1);
  m.text[sizeof(m.text) - 1] = '\0';
  snprintf(m.time_str, sizeof(m.time_str), "%02d:%02d", hr, mn);
  m.is_incoming = is_incoming;

  bt_log_dirty = true;
}

void refresh_bt_message_list() {
  if (!bt_log_label) return;

  if (bt_msg_count == 0) {
    lv_label_set_text(bt_log_label,
      "Android BLE Ready (" BLE_DEVICE_NAME ")\n"
      "Supported Android Commands:\n"
      "- TIME:YYYY-MM-DD HH:MM:SS\n"
      "- REQ:<question> | CALL:<name>\n"
      "- NOTIFY:<title>|<message>\n"
      "- WEATHER:<c>,<cond>,<city>\n"
      "- FACE:0-6 | BRIGHT:0-100\n"
      "- DIAG | STATUS | FIND");
    return;
  }

  static char full_log_buf[1536];
  size_t pos = 0;
  full_log_buf[0] = '\0';

  for (int i = bt_msg_count - 1; i >= 0; i--) {
    int written = snprintf(
      full_log_buf + pos,
      sizeof(full_log_buf) - pos,
      "%s[%s] %s: %s",
      (i == bt_msg_count - 1) ? "" : "\n",
      bt_messages[i].time_str,
      bt_messages[i].is_incoming ? "IN" : "OUT",
      bt_messages[i].text
    );
    if (written <= 0) break;
    if ((size_t)written >= sizeof(full_log_buf) - pos) {
      pos = sizeof(full_log_buf) - 1;
      break;
    }
    pos += (size_t)written;
  }
  lv_label_set_text(bt_log_label, full_log_buf);
}

void send_ble_message(const char* text, bool log_to_ui) {
  if (!text || text[0] == '\0') return;

  if (ble_power_on && ble_initialized && ble_tx_char) {
    char payload[180];
    snprintf(payload, sizeof(payload), "%s\n", text);
    ble_tx_char->setValue((uint8_t*)payload, strlen(payload));
    // Only call notify() if connected AND after Android finishes GATT service discovery/MTU setup (>800ms)
    if (ble_device_connected && (millis() - ble_connect_event_ms >= 800)) {
      bool cccd_enabled = (ble_tx_cccd != NULL) ? ble_tx_cccd->getNotifications() : true;
      if (cccd_enabled && (millis() - last_ble_notify_ms >= 20)) {
        last_ble_notify_ms = millis();
        ble_tx_char->notify();
      }
    }
  }

  if (log_to_ui) {
    add_bt_message_log(text, false);
  }
}

void request_android_ble_sync() {
  int yr, mo, dy, hr, mn, sc;
  get_watch_datetime(yr, mo, dy, hr, mn, sc);
  int pct = (pmu_available && power.isBatteryConnect()) ? power.getBatteryPercent() : 100;
  char req[128];
  snprintf(req, sizeof(req), "REQ:SYNC TIME=%04d-%02d-%02d_%02d:%02d:%02d BATT=%d%%", yr, mo, dy, hr, mn, sc, pct);
  send_ble_message(req, true);
  if (ntp_status_label) {
    lv_label_set_text(ntp_status_label, ble_device_connected ? "BT Sync Requested from Android" : "Connect Android BLE to Sync");
  }
}

// --- Interactive Android Request Modal (Accept / Reject on Watch) ---
void open_android_request_modal(const char* title_str, const char* body_str, const char* accept_reply, const char* reject_reply) {
  wake_screen();
  play_beep(1100, 70);

  if (bt_req_modal != NULL) {
    lv_obj_delete_async(bt_req_modal);
    bt_req_modal = NULL;
  }

  strncpy(pending_android_accept_reply, accept_reply ? accept_reply : "RESP:ACCEPTED", sizeof(pending_android_accept_reply) - 1);
  pending_android_accept_reply[sizeof(pending_android_accept_reply) - 1] = '\0';
  strncpy(pending_android_reject_reply, reject_reply ? reject_reply : "RESP:REJECTED", sizeof(pending_android_reject_reply) - 1);
  pending_android_reject_reply[sizeof(pending_android_reject_reply) - 1] = '\0';

  bt_req_modal = lv_obj_create(lv_scr_act());
  lv_obj_set_size(bt_req_modal, 336, 236);
  lv_obj_center(bt_req_modal);
  lv_obj_set_style_bg_color(bt_req_modal, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(bt_req_modal, lv_color_hex(0xCBA6F7), 0);
  lv_obj_set_style_border_width(bt_req_modal, 3, 0);
  lv_obj_set_style_radius(bt_req_modal, 24, 0);
  lv_obj_set_style_pad_all(bt_req_modal, 10, 0);
  lv_obj_remove_flag(bt_req_modal, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * t_lbl = lv_label_create(bt_req_modal);
  lv_label_set_text(t_lbl, title_str);
  lv_obj_set_style_text_font(t_lbl, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(t_lbl, lv_color_hex(0xCBA6F7), 0);
  lv_obj_align(t_lbl, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t * b_lbl = lv_label_create(bt_req_modal);
  lv_label_set_text(b_lbl, body_str);
  lv_obj_set_width(b_lbl, 304);
  lv_label_set_long_mode(b_lbl, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(b_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(b_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(b_lbl, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(b_lbl, LV_ALIGN_CENTER, 0, -4);

  lv_obj_t * btn_accept = lv_button_create(bt_req_modal);
  lv_obj_set_size(btn_accept, 142, 42);
  lv_obj_align(btn_accept, LV_ALIGN_BOTTOM_LEFT, 4, -2);
  lv_obj_set_style_bg_color(btn_accept, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_radius(btn_accept, 16, 0);
  lv_obj_t * lbl_acc = lv_label_create(btn_accept);
  lv_label_set_text(lbl_acc, LV_SYMBOL_OK " Accept");
  lv_obj_set_style_text_color(lbl_acc, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_acc);

  lv_obj_add_event_cb(btn_accept, [](lv_event_t * e) {
    play_beep(1200, 70);
    send_ble_message(pending_android_accept_reply, true);
    if (bt_req_modal) {
      lv_obj_delete_async(bt_req_modal);
      bt_req_modal = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_reject = lv_button_create(bt_req_modal);
  lv_obj_set_size(btn_reject, 142, 42);
  lv_obj_align(btn_reject, LV_ALIGN_BOTTOM_RIGHT, -4, -2);
  lv_obj_set_style_bg_color(btn_reject, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_radius(btn_reject, 16, 0);
  lv_obj_t * lbl_rej = lv_label_create(btn_reject);
  lv_label_set_text(lbl_rej, LV_SYMBOL_CLOSE " Reject");
  lv_obj_set_style_text_color(lbl_rej, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_rej);

  lv_obj_add_event_cb(btn_reject, [](lv_event_t * e) {
    play_beep(450, 70);
    send_ble_message(pending_android_reject_reply, true);
    if (bt_req_modal) {
      lv_obj_delete_async(bt_req_modal);
      bt_req_modal = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);
}

// Helper to extract a JSON string value by key from a lightweight JSON payload
static bool extract_json_str(const char* json, const char* key, char* out, size_t out_len) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p + strlen(pattern), ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p == '\"') {
    p++;
    size_t i = 0;
    while (*p && *p != '\"' && i + 1 < out_len) {
      out[i++] = *p++;
    }
    out[i] = '\0';
    return true;
  } else {
    size_t i = 0;
    while (*p && *p != ',' && *p != '}' && *p != ' ' && i + 1 < out_len) {
      out[i++] = *p++;
    }
    out[i] = '\0';
    return i > 0;
  }
}

static void handle_incoming_ble_payload(const char* raw_in) {
  char clean[256];
  strncpy(clean, raw_in, sizeof(clean) - 1);
  clean[sizeof(clean) - 1] = '\0';

  // Strip trailing CR/LF/spaces
  int len = strlen(clean);
  while (len > 0 && (clean[len - 1] == '\r' || clean[len - 1] == '\n' || clean[len - 1] == ' ')) {
    clean[--len] = '\0';
  }
  if (len == 0) return;

  USBSerial.printf("Android BLE RX: %s\n", clean);
  last_touch_time = millis(); // Keep watch awake during active BLE communication
  snprintf(last_android_req_str, sizeof(last_android_req_str), "Android: %.44s", clean);
  if (wifi_status_label) {
    lv_label_set_text(wifi_status_label, last_android_req_str);
  }

  // 0. If payload is JSON from an Android app (e.g. {"cmd":"time","val":"..."} or {"cmd":"req","msg":"..."})
  if (clean[0] == '{') {
    char cmd_val[32] = "";
    char msg_val[128] = "";
    char title_val[64] = "ANDROID";
    extract_json_str(clean, "cmd", cmd_val, sizeof(cmd_val));
    if (!extract_json_str(clean, "msg", msg_val, sizeof(msg_val))) {
      if (!extract_json_str(clean, "body", msg_val, sizeof(msg_val))) {
        extract_json_str(clean, "val", msg_val, sizeof(msg_val));
      }
    }
    extract_json_str(clean, "title", title_val, sizeof(title_val));

    if (strcasecmp(cmd_val, "time") == 0) {
      char epoch_str[32] = "";
      if (extract_json_str(clean, "epoch", epoch_str, sizeof(epoch_str)) || strlen(msg_val) > 0) {
        char rewritten[96];
        snprintf(rewritten, sizeof(rewritten), "TIME:%s", strlen(epoch_str) > 0 ? epoch_str : msg_val);
        handle_incoming_ble_payload(rewritten);
        return;
      }
    } else if (strcasecmp(cmd_val, "req") == 0 || strcasecmp(cmd_val, "confirm") == 0) {
      add_bt_message_log(clean, true);
      open_android_request_modal(title_val, strlen(msg_val) ? msg_val : "Accept Android request?", "RESP:ACCEPTED", "RESP:REJECTED");
      return;
    } else if (strcasecmp(cmd_val, "notify") == 0 || strcasecmp(cmd_val, "alert") == 0) {
      add_bt_message_log(clean, true);
      trigger_alert_popup(title_val, strlen(msg_val) ? msg_val : clean);
      send_ble_message("OK:NOTIFY", false);
      return;
    } else if (strcasecmp(cmd_val, "call") == 0) {
      add_bt_message_log(clean, true);
      open_android_request_modal("INCOMING CALL", strlen(msg_val) ? msg_val : "Android Call", "CALL:ANSWERED", "CALL:DECLINED");
      return;
    }
  }

  // 1. TIME Sync Request: TIME:YYYY-MM-DD HH:MM:SS, TIME:HH:MM:SS, or TIME:<unix_epoch>
  if (strncasecmp(clean, "TIME:", 5) == 0 || strncasecmp(clean, "SYNC_TIME:", 10) == 0) {
    const char* p = strchr(clean, ':') + 1;
    while (*p == ' ') p++;
    int yr = 0, mo = 0, dy = 0, hr = 0, mn = 0, sc = 0;
    if (sscanf(p, "%d-%d-%d %d:%d:%d", &yr, &mo, &dy, &hr, &mn, &sc) == 6 ||
        sscanf(p, "%d/%d/%d %d:%d:%d", &yr, &mo, &dy, &hr, &mn, &sc) == 6) {
      set_watch_datetime(yr, mo, dy, hr, mn, sc);
      play_beep(1200, 120);
      char ack[80];
      snprintf(ack, sizeof(ack), "OK:TIME_SYNC (%04d-%02d-%02d %02d:%02d:%02d)", yr, mo, dy, hr, mn, sc);
      add_bt_message_log(ack, true);
      send_ble_message(ack, false);
      return;
    } else if (sscanf(p, "%d:%d:%d", &hr, &mn, &sc) >= 2) {
      int cur_y, cur_m, cur_d, cur_h, cur_min, cur_s;
      get_watch_datetime(cur_y, cur_m, cur_d, cur_h, cur_min, cur_s);
      set_watch_datetime(cur_y, cur_m, cur_d, hr, mn, sc);
      play_beep(1200, 120);
      char ack[80];
      snprintf(ack, sizeof(ack), "OK:TIME_SYNC (%02d:%02d:%02d)", hr, mn, sc);
      add_bt_message_log(ack, true);
      send_ble_message(ack, false);
      return;
    } else if (strlen(p) >= 9 && isdigit((unsigned char)p[0])) {
      time_t epoch = (time_t)strtoll(p, NULL, 10);
      if (epoch > 1700000000) {
        struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        struct tm ti;
        localtime_r(&epoch, &ti);
        set_watch_datetime(ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec);
        play_beep(1200, 120);
        char ack[80];
        snprintf(ack, sizeof(ack), "OK:TIME_EPOCH (%04d-%02d-%02d %02d:%02d:%02d)",
                 ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec);
        add_bt_message_log(ack, true);
        send_ble_message(ack, false);
        return;
      }
    }
  }

  // 2. Interactive Request from Android: REQ:<question> or CONFIRM:<question>
  if (strncasecmp(clean, "REQ:", 4) == 0 || strncasecmp(clean, "CONFIRM:", 8) == 0) {
    const char* p = strchr(clean, ':') + 1;
    while (*p == ' ') p++;
    add_bt_message_log(clean, true);
    char acc_buf[64], rej_buf[64];
    snprintf(acc_buf, sizeof(acc_buf), "RESP:ACCEPTED:%.40s", p);
    snprintf(rej_buf, sizeof(rej_buf), "RESP:REJECTED:%.40s", p);
    open_android_request_modal("ANDROID REQUEST", p, acc_buf, rej_buf);
    return;
  }

  // 3. Incoming Call Request from Android: CALL:<caller>
  if (strncasecmp(clean, "CALL:", 5) == 0) {
    const char* p = clean + 5;
    while (*p == ' ') p++;
    add_bt_message_log(clean, true);
    open_android_request_modal("INCOMING CALL", p, "CALL:ANSWERED", "CALL:DECLINED");
    return;
  }

  // 4. Notification / Alert Popup Request: NOTIFY:<Title>|<Body> or NOTIFY:<Body> or ALERT:<Body>
  if (strncasecmp(clean, "NOTIFY:", 7) == 0 || strncasecmp(clean, "ALERT:", 6) == 0) {
    const char* p = strchr(clean, ':') + 1;
    while (*p == ' ') p++;
    char title_buf[48] = "ANDROID ALERT";
    char body_buf[128] = "";
    const char* sep = strchr(p, '|');
    if (sep) {
      size_t tlen = (size_t)(sep - p);
      if (tlen >= sizeof(title_buf)) tlen = sizeof(title_buf) - 1;
      memcpy(title_buf, p, tlen);
      title_buf[tlen] = '\0';
      strncpy(body_buf, sep + 1, sizeof(body_buf) - 1);
    } else {
      strncpy(body_buf, p, sizeof(body_buf) - 1);
    }
    add_bt_message_log(clean, true);
    trigger_alert_popup(title_buf, body_buf);
    send_ble_message("OK:NOTIFY", false);
    return;
  }

  // 5. Weather Sync Request over Bluetooth: WEATHER:<tempC>,<condition>,<city>[,<humidity>,<wind>]
  if (strncasecmp(clean, "WEATHER:", 8) == 0) {
    const char* p = clean + 8;
    while (*p == ' ') p++;
    char wcopy[128];
    strncpy(wcopy, p, sizeof(wcopy) - 1);
    wcopy[sizeof(wcopy) - 1] = '\0';

    char *tok_temp = strtok(wcopy, ",");
    char *tok_cond = strtok(NULL, ",");
    char *tok_city = strtok(NULL, ",");
    char *tok_hum  = strtok(NULL, ",");
    char *tok_wind = strtok(NULL, ",");

    if (tok_temp) {
      weather_data.temp_c = atof(tok_temp);
      weather_data.temp_f = (weather_data.temp_c * 9.0f / 5.0f) + 32.0f;
      weather_data.temp_max_c = weather_data.temp_c + 3.0f;
      weather_data.temp_min_c = weather_data.temp_c - 3.0f;
      weather_data.temp_max_f = (weather_data.temp_max_c * 9.0f / 5.0f) + 32.0f;
      weather_data.temp_min_f = (weather_data.temp_min_c * 9.0f / 5.0f) + 32.0f;
      if (tok_cond && strlen(tok_cond) > 0) {
        strncpy(weather_data.condition_str, tok_cond, sizeof(weather_data.condition_str) - 1);
      }
      if (tok_city && strlen(tok_city) > 0) {
        strncpy(weather_data.city_str, tok_city, sizeof(weather_data.city_str) - 1);
      }
      if (tok_hum) weather_data.humidity = atoi(tok_hum);
      if (tok_wind) weather_data.wind_speed = atof(tok_wind);
      weather_data.valid = true;
      weather_data.last_update_ms = millis();
      snprintf(weather_status_msg, sizeof(weather_status_msg), "Synced via Android BT");
      update_weather_display();
      play_beep(1050, 80);
      add_bt_message_log(clean, true);
      send_ble_message("OK:WEATHER_UPDATED", false);
      return;
    }
  }

  // 6. Fitness / Sport Stats from Android: STEPS:<count> or HR:<bpm> / PULSE:<bpm>
  if (strncasecmp(clean, "STEPS:", 6) == 0) {
    watch_steps_count = (uint32_t)strtoul(clean + 6, NULL, 10);
    last_clock_update = 0;
    update_clock_display();
    add_bt_message_log(clean, true);
    send_ble_message("OK:STEPS_UPDATED", false);
    return;
  }
  if (strncasecmp(clean, "HR:", 3) == 0 || strncasecmp(clean, "PULSE:", 6) == 0) {
    const char* p = strchr(clean, ':') + 1;
    watch_heart_rate_bpm = atoi(p);
    last_clock_update = 0;
    update_clock_display();
    add_bt_message_log(clean, true);
    send_ble_message("OK:HR_UPDATED", false);
    return;
  }

  // 7. Remote Watch Face Change: FACE:<0-6> or FACE:NEXT
  if (strncasecmp(clean, "FACE:", 5) == 0) {
    const char* p = clean + 5;
    while (*p == ' ') p++;
    int next_f = selected_face_idx;
    if (strcasecmp(p, "NEXT") == 0) {
      next_f = (selected_face_idx + 1) % num_faces;
    } else {
      next_f = atoi(p);
    }
    if (next_f >= 0 && next_f < num_faces) {
      wake_screen();
      switch_to_page(0);
      select_watch_face(next_f);
      char resp[64];
      snprintf(resp, sizeof(resp), "OK:FACE=%d (%s)", next_f, face_names[next_f]);
      add_bt_message_log(clean, true);
      send_ble_message(resp, false);
      return;
    }
  }

  // 8. Remote Page Switch: PAGE:<0-3>
  if (strncasecmp(clean, "PAGE:", 5) == 0) {
    int pg = atoi(clean + 5);
    if (pg >= 0 && pg < NUM_PAGES) {
      wake_screen();
      switch_to_page(pg);
      char resp[48];
      snprintf(resp, sizeof(resp), "OK:PAGE=%d", pg);
      add_bt_message_log(clean, true);
      send_ble_message(resp, false);
      return;
    }
  }

  // 9. Remote Brightness Control: BRIGHT:<0-100> or BRIGHTNESS:<0-100>
  if (strncasecmp(clean, "BRIGHT:", 7) == 0 || strncasecmp(clean, "BRIGHTNESS:", 11) == 0) {
    const char* p = strchr(clean, ':') + 1;
    int pct = atoi(p);
    if (pct < 10) pct = 10;
    if (pct > 100) pct = 100;
    uint8_t raw_b = (uint8_t)((pct * 255) / 100);
    set_amoled_brightness(raw_b);
    if (brightness_slider) lv_slider_set_value(brightness_slider, raw_b, LV_ANIM_OFF);
    if (brightness_lbl_val) {
      char bbuf[32];
      snprintf(bbuf, sizeof(bbuf), "Brightness: %d%%", pct);
      lv_label_set_text(brightness_lbl_val, bbuf);
    }
    char resp[48];
    snprintf(resp, sizeof(resp), "OK:BRIGHT=%d%%", pct);
    add_bt_message_log(clean, true);
    send_ble_message(resp, false);
    return;
  }

  // 10. Remote Alarm Configuration: ALARM:HH:MM or ALARM:OFF or ALARM:ON
  if (strncasecmp(clean, "ALARM:", 6) == 0) {
    const char* p = clean + 6;
    while (*p == ' ') p++;
    int ah = 0, am = 0;
    if (strcasecmp(p, "OFF") == 0) {
      alarm_enabled = false;
      prefs.putBool("alarm_en", false);
      if (alarm_sw_enable) lv_obj_clear_state(alarm_sw_enable, LV_STATE_CHECKED);
      add_bt_message_log(clean, true);
      send_ble_message("OK:ALARM_OFF", false);
      return;
    } else if (strcasecmp(p, "ON") == 0) {
      alarm_enabled = true;
      prefs.putBool("alarm_en", true);
      if (alarm_sw_enable) lv_obj_add_state(alarm_sw_enable, LV_STATE_CHECKED);
      add_bt_message_log(clean, true);
      send_ble_message("OK:ALARM_ON", false);
      return;
    } else if (sscanf(p, "%d:%d", &ah, &am) == 2) {
      alarm_hour = (ah + 24) % 24;
      alarm_min = (am + 60) % 60;
      alarm_enabled = true;
      prefs.putInt("alarm_h", alarm_hour);
      prefs.putInt("alarm_m", alarm_min);
      prefs.putBool("alarm_en", true);
      if (alarm_sw_enable) lv_obj_add_state(alarm_sw_enable, LV_STATE_CHECKED);
      if (alarm_lbl_time) {
        char abuf[32];
        int h12 = alarm_hour % 12;
        if (h12 == 0) h12 = 12;
        snprintf(abuf, sizeof(abuf), "%02d:%02d %s", h12, alarm_min, alarm_hour >= 12 ? "PM" : "AM");
        lv_label_set_text(alarm_lbl_time, abuf);
      }
      play_beep(950, 80);
      char resp[64];
      snprintf(resp, sizeof(resp), "OK:ALARM_SET=%02d:%02d", alarm_hour, alarm_min);
      add_bt_message_log(clean, true);
      send_ble_message(resp, false);
      return;
    }
  }

  // 11. Remote Countdown Timer Control: TIMER:<seconds> or TIMER:START / STOP / RESET
  if (strncasecmp(clean, "TIMER:", 6) == 0) {
    const char* p = clean + 6;
    while (*p == ' ') p++;
    if (strcasecmp(p, "STOP") == 0) {
      cd_running = false;
      if (cd_lbl_start) lv_label_set_text(cd_lbl_start, LV_SYMBOL_PLAY " Start");
      send_ble_message("OK:TIMER_STOPPED", false);
      return;
    } else if (strcasecmp(p, "RESET") == 0) {
      cd_running = false;
      cd_duration_ms = 60000;
      cd_remaining_ms = 60000;
      if (cd_lbl_start) lv_label_set_text(cd_lbl_start, LV_SYMBOL_PLAY " Start");
      send_ble_message("OK:TIMER_RESET", false);
      return;
    } else if (strcasecmp(p, "START") == 0) {
      cd_running = true;
      cd_start_time = millis();
      if (cd_lbl_start) lv_label_set_text(cd_lbl_start, LV_SYMBOL_PAUSE " Pause");
      send_ble_message("OK:TIMER_STARTED", false);
      return;
    } else {
      int sec_val = atoi(p);
      if (sec_val > 0) {
        cd_duration_ms = (uint32_t)sec_val * 1000UL;
        cd_remaining_ms = cd_duration_ms;
        cd_start_time = millis();
        cd_running = true;
        if (cd_lbl_start) lv_label_set_text(cd_lbl_start, LV_SYMBOL_PAUSE " Pause");
        play_beep(1000, 80);
        char resp[64];
        snprintf(resp, sizeof(resp), "OK:TIMER_STARTED=%ds", sec_val);
        add_bt_message_log(clean, true);
        send_ble_message(resp, false);
        return;
      }
    }
  }

  // 12. Find Watch / Ring / Beep Request
  if (strcasecmp(clean, "FIND") == 0 || strcasecmp(clean, "RING") == 0 || strcasecmp(clean, "BEEP") == 0) {
    wake_screen();
    play_beep(880, 120);
    play_beep(1175, 120);
    play_beep(1568, 180);
    add_bt_message_log(clean, true);
    trigger_alert_popup("FIND MY WATCH", "Pinged from your Android phone!");
    send_ble_message("OK:WATCH_FOUND", false);
    return;
  }

  // 13. PING / STATUS / INFO / DIAG / BOUNDARIES / HELP Request
  if (strcasecmp(clean, "DIAG") == 0 || strcasecmp(clean, "BOUNDARIES") == 0) {
    char resp[168];
    snprintf(resp, sizeof(resp),
             "BOUNDARIES IntSRAM:%uKB(blk:%uKB,min:%uKB) PSRAM:%uKB Stack:%uB MaxLoop:%lums Fails:%lu",
             (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned int)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned int)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned int)uxTaskGetStackHighWaterMark(NULL),
             (unsigned long)boundary_max_loop_ms,
             (unsigned long)boundary_alloc_fail_count);
    add_bt_message_log(clean, true);
    send_ble_message(resp, true);
    return;
  }

  if (strcasecmp(clean, "PING") == 0 || strcasecmp(clean, "STATUS") == 0 || strcasecmp(clean, "INFO") == 0) {
    int yr, mo, dy, hr, mn, sc;
    get_watch_datetime(yr, mo, dy, hr, mn, sc);
    int pct = (pmu_available && power.isBatteryConnect()) ? power.getBatteryPercent() : 100;
    uint16_t mv = pmu_available ? power.getBattVoltage() : 4100;
    char resp[140];
    snprintf(resp, sizeof(resp), "STATUS %04d-%02d-%02d %02d:%02d:%02d BATT:%d%%(%umV) FACE:%d STEPS:%lu",
             yr, mo, dy, hr, mn, sc, pct, mv, selected_face_idx, (unsigned long)watch_steps_count);
    add_bt_message_log(clean, true);
    send_ble_message(resp, true);
    return;
  }

  if (strcasecmp(clean, "HELP") == 0) {
    send_ble_message("CMDS: TIME:YYYY-MM-DD HH:MM:SS | REQ:msg | CALL:name | NOTIFY:title|msg | WEATHER:c,cond,city | FACE:0-6 | BRIGHT:0-100 | DIAG | FIND | STATUS", false);
    return;
  }

  // 14. Default: Standard chat/notification message from Android
  wake_screen();
  play_beep(1046, 60);
  add_bt_message_log(clean, true);

  char ack[80];
  snprintf(ack, sizeof(ack), "ACK: %.56s", clean);
  send_ble_message(ack, false);
}

void update_ble_status() {
  uint32_t now = millis();

  // Handle connection state transitions and deferred sync/advertising on Core 1
  if (ble_power_on && ble_initialized) {
    if (!ble_device_connected && ble_old_connected) {
      ble_old_connected = false;
      ble_sync_pending = false;
      ble_adv_restart_pending = true;
      USBSerial.printf("Android BLE Disconnected! IntSRAM=%uKB (largest=%uKB)\n",
                       (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                       (unsigned int)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
      play_beep(500, 45);
    } else if (ble_device_connected && !ble_old_connected) {
      ble_old_connected = true;
      ble_adv_restart_pending = false;
      ble_sync_pending = true;
      USBSerial.printf("Android BLE Connected! Waiting 1.8s for GATT discovery... IntSRAM=%uKB (largest=%uKB)\n",
                       (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                       (unsigned int)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
      play_beep(1100, 45);
    }

    // Restart advertising safely on Core 1 after Bluedroid finishes disconnect teardown (250ms)
    if (ble_adv_restart_pending && !ble_device_connected && (now - ble_disconnect_event_ms >= 250)) {
      ble_adv_restart_pending = false;
      BLEDevice::startAdvertising();
      USBSerial.println("BLE Advertising restarted cleanly.");
    }

    // Defer initial sync notification until Android finishes GATT service discovery & CCCD write (1800ms)
    if (ble_sync_pending && ble_device_connected && (now - ble_connect_event_ms >= 1800)) {
      ble_sync_pending = false;
      request_android_ble_sync();
    }

    if (ble_manual_sync_pending) {
      ble_manual_sync_pending = false;
      request_android_ble_sync();
    }

    // Periodic 15-second Battery Service (0x180F) GATT value update to keep Android link warm
    if (ble_device_connected && (now - last_ble_keepalive_ms >= 15000)) {
      last_ble_keepalive_ms = now;
      if (ble_batt_char) {
        uint8_t pct = (uint8_t)((pmu_available && power.isBatteryConnect()) ? power.getBatteryPercent() : 100);
        ble_batt_char->setValue(&pct, 1);
      }
    }
  } else {
    ble_manual_sync_pending = false;
  }

  // Drain at most 2 incoming BLE RX messages per loop pass so UI rendering is never starved
  int processed = 0;
  while (processed < 2) {
    char msg_copy[192];
    bool has_msg = false;
    portENTER_CRITICAL(&ble_rx_mux);
    if (ble_rx_tail != ble_rx_head) {
      strncpy(msg_copy, ble_rx_queue[ble_rx_tail], sizeof(msg_copy) - 1);
      msg_copy[sizeof(msg_copy) - 1] = '\0';
      ble_rx_tail = (ble_rx_tail + 1) % BLE_RX_QUEUE_SIZE;
      has_msg = true;
    }
    portEXIT_CRITICAL(&ble_rx_mux);

    if (!has_msg) break;
    handle_incoming_ble_payload(msg_copy);
    processed++;
  }

  if (bt_log_dirty && active_page_idx == 2) {
    bt_log_dirty = false;
    refresh_bt_message_list();
  }

  // Update BLE UI labels ONLY when state or message count changes (avoids 200Hz LVGL dirty invalidations)
  int cur_ui_state = !ble_power_on ? 0 : (ble_device_connected ? 2 : 1);
  if (cur_ui_state != last_ble_ui_state || bt_msg_count != last_ble_ui_msg_count) {
    last_ble_ui_state = cur_ui_state;
    last_ble_ui_msg_count = bt_msg_count;

    if (bt_status_lbl) {
      if (cur_ui_state == 0) {
        lv_label_set_text(bt_status_lbl, "Bluetooth: OFF");
        lv_obj_set_style_text_color(bt_status_lbl, lv_color_hex(0xF38BA8), 0);
      } else if (cur_ui_state == 2) {
        lv_label_set_text(bt_status_lbl, "Connected: " BLE_DEVICE_NAME);
        lv_obj_set_style_text_color(bt_status_lbl, lv_color_hex(0xA6E3A1), 0);
      } else {
        lv_label_set_text(bt_status_lbl, "Advertising: " BLE_DEVICE_NAME);
        lv_obj_set_style_text_color(bt_status_lbl, lv_color_hex(0x89B4FA), 0);
      }
    }

    if (bt_clock_status_label) {
      if (cur_ui_state == 0) {
        lv_label_set_text(bt_clock_status_label, "Bluetooth: Off");
      } else if (cur_ui_state == 2) {
        char bbuf[64];
        snprintf(bbuf, sizeof(bbuf), "Android BLE: Connected (%d msgs)", bt_msg_count);
        lv_label_set_text(bt_clock_status_label, bbuf);
      } else {
        lv_label_set_text(bt_clock_status_label, "BLE Ready (" BLE_DEVICE_NAME ")");
      }
    }
  }
}

void open_bt_compose_modal() {
  if (bt_compose_modal != NULL) return;
  play_beep(900, 50);

  bt_compose_modal = lv_obj_create(lv_scr_act());
  lv_obj_set_size(bt_compose_modal, SAFE_MODAL_W, 380);
  lv_obj_center(bt_compose_modal);
  lv_obj_set_style_bg_color(bt_compose_modal, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(bt_compose_modal, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_border_width(bt_compose_modal, 2, 0);
  lv_obj_set_style_radius(bt_compose_modal, 24, 0);
  lv_obj_set_style_pad_all(bt_compose_modal, 8, 0);
  lv_obj_remove_flag(bt_compose_modal, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * title = lv_label_create(bt_compose_modal);
  lv_label_set_text(title, "Send Bluetooth Message");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0x89B4FA), 0);

  bt_compose_ta = lv_textarea_create(bt_compose_modal);
  lv_textarea_set_one_line(bt_compose_ta, true);
  lv_textarea_set_placeholder_text(bt_compose_ta, "Type message...");
  lv_obj_set_size(bt_compose_ta, 312, 40);
  lv_obj_align(bt_compose_ta, LV_ALIGN_TOP_MID, 0, 26);

  lv_obj_t * kb = lv_keyboard_create(bt_compose_modal);
  lv_obj_set_size(kb, 318, 195);
  lv_obj_align(kb, LV_ALIGN_TOP_MID, 0, 74);
  lv_keyboard_set_textarea(kb, bt_compose_ta);

  lv_obj_t * btn_send = lv_button_create(bt_compose_modal);
  lv_obj_set_size(btn_send, 144, 38);
  lv_obj_align(btn_send, LV_ALIGN_BOTTOM_LEFT, 6, -4);
  lv_obj_set_style_bg_color(btn_send, lv_color_hex(0xA6E3A1), 0);
  lv_obj_t * lbl_send = lv_label_create(btn_send);
  lv_label_set_text(lbl_send, LV_SYMBOL_UPLOAD " Send");
  lv_obj_set_style_text_color(lbl_send, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_send);

  lv_obj_add_event_cb(btn_send, [](lv_event_t * e) {
    play_beep(1000, 60);
    char msg_buf[128] = "";
    if (bt_compose_ta) {
      const char * txt = lv_textarea_get_text(bt_compose_ta);
      if (txt) {
        strncpy(msg_buf, txt, sizeof(msg_buf) - 1);
        msg_buf[sizeof(msg_buf) - 1] = '\0';
      }
    }
    if (bt_compose_modal) {
      lv_obj_delete_async(bt_compose_modal);
      bt_compose_modal = NULL;
      bt_compose_ta = NULL;
    }
    if (strlen(msg_buf) > 0) {
      send_ble_message(msg_buf, true);
    }
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_cancel = lv_button_create(bt_compose_modal);
  lv_obj_set_size(btn_cancel, 144, 38);
  lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_RIGHT, -6, -4);
  lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0xF38BA8), 0);
  lv_obj_t * lbl_cancel = lv_label_create(btn_cancel);
  lv_label_set_text(lbl_cancel, LV_SYMBOL_CLOSE " Cancel");
  lv_obj_set_style_text_color(lbl_cancel, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_cancel);

  lv_obj_add_event_cb(btn_cancel, [](lv_event_t * e) {
    play_beep(450, 50);
    if (bt_compose_modal) {
      lv_obj_delete_async(bt_compose_modal);
      bt_compose_modal = NULL;
      bt_compose_ta = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);
}

// --- Wi-Fi Power Control (Kept OFF while Wi-Fi Menu is Disabled) ---
void set_wifi_power(bool enable) {
  (void)enable;
  wifi_power_on = false;
  wifi_connect_in_progress = false;
  wifi_scan_in_progress = false;
  ntp_sync_in_progress = false;
}

void update_wifi_connect_status() {
  // Wi-Fi menu removed; no-op
}

// --- BLUETOOTH SYNC HELPER ---
void start_ntp_sync() {
  request_android_ble_sync();
}

void update_ntp_sync_status() {
  // Managed via Android BLE TIME sync
}

// --- Select Watch Face ---
void select_watch_face(int face_idx) {
  if (face_idx < 0 || face_idx >= num_faces) return;
  if (selected_face_idx != face_idx) {
    selected_face_idx = face_idx;
    prefs.putInt("face_idx", selected_face_idx);
    play_beep(900, 45);
  }

  // Hide all watch face containers
  if (face_container_modern) lv_obj_add_flag(face_container_modern, LV_OBJ_FLAG_HIDDEN);
  if (face_container_sundial) lv_obj_add_flag(face_container_sundial, LV_OBJ_FLAG_HIDDEN);
  if (face_container_neon) lv_obj_add_flag(face_container_neon, LV_OBJ_FLAG_HIDDEN);
  if (face_container_analog) lv_obj_add_flag(face_container_analog, LV_OBJ_FLAG_HIDDEN);
  if (face_container_ascii) lv_obj_add_flag(face_container_ascii, LV_OBJ_FLAG_HIDDEN);
  if (face_container_weather) lv_obj_add_flag(face_container_weather, LV_OBJ_FLAG_HIDDEN);
  if (face_container_amoled_arc) lv_obj_add_flag(face_container_amoled_arc, LV_OBJ_FLAG_HIDDEN);

  // Sync settings dropdown if present
  if (settings_face_dropdown && lv_dropdown_get_selected(settings_face_dropdown) != selected_face_idx) {
    lv_dropdown_set_selected(settings_face_dropdown, selected_face_idx);
  }

  // Show active face container
  if (selected_face_idx == 0 && face_container_modern) {
    lv_obj_clear_flag(face_container_modern, LV_OBJ_FLAG_HIDDEN);
  } else if (selected_face_idx == 1 && face_container_sundial) {
    lv_obj_clear_flag(face_container_sundial, LV_OBJ_FLAG_HIDDEN);
    update_sundial_display();
  } else if (selected_face_idx == 2 && face_container_neon) {
    lv_obj_clear_flag(face_container_neon, LV_OBJ_FLAG_HIDDEN);
  } else if (selected_face_idx == 3 && face_container_analog) {
    lv_obj_clear_flag(face_container_analog, LV_OBJ_FLAG_HIDDEN);
  } else if (selected_face_idx == 4 && face_container_ascii) {
    lv_obj_clear_flag(face_container_ascii, LV_OBJ_FLAG_HIDDEN);
  } else if (selected_face_idx == 5 && face_container_weather) {
    lv_obj_clear_flag(face_container_weather, LV_OBJ_FLAG_HIDDEN);
    update_weather_display();
  } else if (selected_face_idx == 6 && face_container_amoled_arc) {
    lv_obj_clear_flag(face_container_amoled_arc, LV_OBJ_FLAG_HIDDEN);
    last_clock_update = 0;
    update_clock_display();
  }
}

// --- Page Switching Logic (0=Clock, 1=Tools, 2=Bluetooth, 3=Settings) ---
void switch_to_page(int page_idx) {
  if (page_idx < 0 || page_idx >= NUM_PAGES) page_idx = 0;
  active_page_idx = page_idx;
  play_beep(800, 40);

  if (page_clock) lv_obj_add_flag(page_clock, LV_OBJ_FLAG_HIDDEN);
  if (page_tools) lv_obj_add_flag(page_tools, LV_OBJ_FLAG_HIDDEN);
  if (page_bt) lv_obj_add_flag(page_bt, LV_OBJ_FLAG_HIDDEN);
  if (page_settings) lv_obj_add_flag(page_settings, LV_OBJ_FLAG_HIDDEN);

  if (bar_btn_clock) lv_obj_set_style_bg_color(bar_btn_clock, lv_color_hex(0x313244), 0);
  if (bar_btn_tools) lv_obj_set_style_bg_color(bar_btn_tools, lv_color_hex(0x313244), 0);
  if (bar_btn_bt) lv_obj_set_style_bg_color(bar_btn_bt, lv_color_hex(0x313244), 0);
  if (bar_btn_settings) lv_obj_set_style_bg_color(bar_btn_settings, lv_color_hex(0x313244), 0);

  if (page_idx == 0 && page_clock) {
    lv_obj_clear_flag(page_clock, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(bar_btn_clock, lv_color_hex(0x89B4FA), 0);
    select_watch_face(selected_face_idx);
  } else if (page_idx == 1 && page_tools) {
    lv_obj_clear_flag(page_tools, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(bar_btn_tools, lv_color_hex(0xA6E3A1), 0);
  } else if (page_idx == 2 && page_bt) {
    if (bt_log_dirty) {
      bt_log_dirty = false;
      refresh_bt_message_list();
    }
    lv_obj_clear_flag(page_bt, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(bar_btn_bt, lv_color_hex(0xCBA6F7), 0);
  } else if (page_idx == 3 && page_settings) {
    lv_obj_clear_flag(page_settings, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(bar_btn_settings, lv_color_hex(0xCF5C5C), 0);
  }
}

static void nav_to_page_cb(lv_event_t * e) {
  uint32_t idx = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
  switch_to_page((int)idx);
}

// --- Touch Swipe Gesture Handler ---
static void screen_gesture_event_cb(lv_event_t * e) {
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
  if (dir == LV_DIR_LEFT) {
    play_beep(900, 30);
    int next_page = (active_page_idx + 1) % NUM_PAGES;
    switch_to_page(next_page);
  } else if (dir == LV_DIR_RIGHT) {
    play_beep(700, 30);
    int prev_page = (active_page_idx + NUM_PAGES - 1) % NUM_PAGES;
    switch_to_page(prev_page);
  }
}

// --- Watch Face Long Press Handler ---
static void watch_face_long_press_cb(lv_event_t * e) {
  open_watch_face_gallery();
}

// --- Interactive Watch Face Gallery Modal ---
void open_watch_face_gallery() {
  if (gallery_modal != NULL) return;
  play_beep(1000, 50);

  gallery_modal = lv_obj_create(lv_scr_act());
  lv_obj_set_size(gallery_modal, SAFE_MODAL_W, SAFE_MODAL_H);
  lv_obj_center(gallery_modal);
  lv_obj_set_style_bg_color(gallery_modal, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(gallery_modal, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_border_width(gallery_modal, 3, 0);
  lv_obj_set_style_radius(gallery_modal, 26, 0);
  // Use opaque background (LV_OPA_COVER) to avoid 24KB software alpha layer allocation in Internal SRAM
  lv_obj_set_style_bg_opa(gallery_modal, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(gallery_modal, 10, 0);
  lv_obj_set_style_pad_row(gallery_modal, 6, 0);
  lv_obj_set_flex_flow(gallery_modal, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(gallery_modal, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  lv_obj_t * title = lv_label_create(gallery_modal);
  lv_label_set_text(title, "WATCH FACE GALLERY");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0x89B4FA), 0);

  lv_obj_t * sub = lv_label_create(gallery_modal);
  lv_label_set_text(sub, "Tap a card to activate watch face");
  lv_obj_set_style_text_font(sub, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(sub, lv_color_hex(0xA6ADC8), 0);

  const char* face_titles[] = {
    "0: Modern Digital",
    "1: Sundial Simulator",
    "2: Neon Cyberpunk",
    "3: Analog Classic",
    "4: ASCII Matrix Terminal",
    "5: Weather Station",
    "6: AMOLED Sport Arc"
  };

  const char* face_descs[] = {
    "Digital clock with timezone & battery",
    "Solar arc tracker with scrub mode",
    "Cyberpunk HUD with live seconds bar",
    "Classic dial with hour, min & sec hands",
    "Matrix terminal with ASCII block digits",
    "Bluetooth weather station & forecast",
    "AMOLED top battery arc with steps & HR"
  };

  for (int i = 0; i < num_faces; i++) {
    lv_obj_t * card = lv_button_create(gallery_modal);
    lv_obj_set_size(card, 312, 64);
    lv_obj_set_style_radius(card, 14, 0);
    if (i == selected_face_idx) {
      lv_obj_set_style_bg_color(card, lv_color_hex(0x313244), 0);
      lv_obj_set_style_border_color(card, lv_color_hex(0x89B4FA), 0);
      lv_obj_set_style_border_width(card, 2, 0);
    } else {
      lv_obj_set_style_bg_color(card, lv_color_hex(0x1E1E2E), 0);
      lv_obj_set_style_border_width(card, 0, 0);
    }

    lv_obj_t * card_lbl = lv_label_create(card);
    lv_label_set_text(card_lbl, face_titles[i]);
    lv_obj_set_style_text_font(card_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(card_lbl, i == selected_face_idx ? lv_color_hex(0x89B4FA) : lv_color_hex(0xCDD6F4), 0);
    lv_obj_align(card_lbl, LV_ALIGN_TOP_LEFT, 6, 4);

    lv_obj_t * card_desc = lv_label_create(card);
    lv_label_set_text(card_desc, face_descs[i]);
    lv_obj_set_style_text_font(card_desc, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(card_desc, lv_color_hex(0xA6ADC8), 0);
    lv_obj_align(card_desc, LV_ALIGN_BOTTOM_LEFT, 6, -4);

    if (i == selected_face_idx) {
      lv_obj_t * active_badge = lv_label_create(card);
      lv_label_set_text(active_badge, "ACTIVE");
      lv_obj_set_style_text_font(active_badge, &lv_font_montserrat_14, 0);
      lv_obj_set_style_text_color(active_badge, lv_color_hex(0xA6E3A1), 0);
      lv_obj_align(active_badge, LV_ALIGN_TOP_RIGHT, -6, 4);
    }

    lv_obj_add_event_cb(card, [](lv_event_t * e) {
      int idx = (int)(uintptr_t)lv_event_get_user_data(e);
      select_watch_face(idx);
      if (gallery_modal) {
        lv_obj_delete_async(gallery_modal);
        gallery_modal = NULL;
      }
    }, LV_EVENT_CLICKED, (void*)(uintptr_t)i);
  }

  // Close Button
  lv_obj_t * btn_close = lv_button_create(gallery_modal);
  lv_obj_set_size(btn_close, 312, 38);
  lv_obj_set_style_bg_color(btn_close, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_radius(btn_close, 16, 0);
  lv_obj_t * lbl_close = lv_label_create(btn_close);
  lv_label_set_text(lbl_close, LV_SYMBOL_CLOSE "  Close Gallery");
  lv_obj_set_style_text_color(lbl_close, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_close);

  lv_obj_add_event_cb(btn_close, [](lv_event_t * e) {
    play_beep(500, 50);
    if (gallery_modal) {
      lv_obj_delete_async(gallery_modal);
      gallery_modal = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);
}

// --- UI Event Handlers ---
static void face_dropdown_event_cb(lv_event_t * e) {
  lv_obj_t * dropdown = (lv_obj_t *)lv_event_get_target(e);
  uint16_t sel = lv_dropdown_get_selected(dropdown);
  select_watch_face((int)sel);
}

static void btn_sleep_event_cb(lv_event_t * e) {
  play_beep(400, 80);
  sleep_screen();
}

static void btn_sync_event_cb(lv_event_t * e) {
  ble_manual_sync_pending = true;
  switch_to_page(2);
}

static void wifi_switch_event_cb(lv_event_t * e) {
  play_beep(700, 50);
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
  set_wifi_power(state);
}

static void bt_switch_event_cb(lv_event_t * e) {
  play_beep(750, 50);
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
  set_ble_power(state);
}

static void format_12h_switch_event_cb(lv_event_t * e) {
  play_beep(700, 50);
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  use_12h_format = lv_obj_has_state(sw, LV_STATE_CHECKED);
  prefs.putBool("use_12h", use_12h_format);
  last_clock_update = 0;
  update_clock_display();
}

static void audio_switch_event_cb(lv_event_t * e) {
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  audio_muted = !lv_obj_has_state(sw, LV_STATE_CHECKED);
  prefs.putBool("muted", audio_muted);
  if (!audio_muted) {
    play_beep(800, 80);
  }
}

static void brightness_slider_event_cb(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  int val = lv_slider_get_value(slider);
  set_amoled_brightness((uint8_t)val);

  if (brightness_lbl_val) {
    char bbuf[32];
    int pct = (val * 100) / 255;
    snprintf(bbuf, sizeof(bbuf), "Brightness: %d%%", pct);
    lv_label_set_text(brightness_lbl_val, bbuf);
  }
}

static void tz_dropdown_event_cb(lv_event_t * e) {
  play_beep(650, 40);
  lv_obj_t * dropdown = (lv_obj_t *)lv_event_get_target(e);
  uint16_t sel = lv_dropdown_get_selected(dropdown);
  if (sel < num_tz_options) {
    current_tz_idx = sel;
    prefs.putInt("tz_idx", current_tz_idx);
    last_clock_update = 0;
    update_clock_display();
  }
}

// --- Stopwatch Controls ---
static void sw_btn_start_cb(lv_event_t * e) {
  play_beep(sw_running ? 500 : 1000, 80);
  if (sw_running) {
    sw_running = false;
    sw_accumulated_ms += (millis() - sw_start_time);
    lv_label_set_text(sw_lbl_start, LV_SYMBOL_PLAY " Start");
  } else {
    sw_running = true;
    sw_start_time = millis();
    lv_label_set_text(sw_lbl_start, LV_SYMBOL_PAUSE " Pause");
  }
}

static void sw_btn_reset_cb(lv_event_t * e) {
  play_beep(400, 100);
  sw_running = false;
  sw_start_time = 0;
  sw_accumulated_ms = 0;
  sw_lap_count = 0;
  if (sw_time_label) lv_label_set_text(sw_time_label, "00:00.0");
  if (sw_lap_lbl) lv_label_set_text(sw_lap_lbl, "Stopwatch Ready");
  if (sw_lbl_start) lv_label_set_text(sw_lbl_start, LV_SYMBOL_PLAY " Start");
  if (sw_lap_list) lv_obj_clean(sw_lap_list);
}

static void sw_btn_lap_cb(lv_event_t * e) {
  play_beep(900, 60);
  uint32_t current_ms = sw_accumulated_ms + (sw_running ? (millis() - sw_start_time) : 0);
  int min_val = (current_ms / 60000) % 60;
  int sec_val = (current_ms / 1000) % 60;
  int tenths = (current_ms / 100) % 10;

  sw_lap_count++;
  char lap_buf[64];
  snprintf(lap_buf, sizeof(lap_buf), "Lap #%d: %02d:%02d.%d", sw_lap_count, min_val, sec_val, tenths);
  if (sw_lap_lbl) lv_label_set_text(sw_lap_lbl, lap_buf);
  if (sw_lap_list) lv_list_add_text(sw_lap_list, lap_buf);
}

// --- Timer Controls ---
static void cd_btn_preset_cb(lv_event_t * e) {
  play_beep(750, 50);
  uint32_t add_ms = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
  cd_duration_ms += add_ms;
  if (!cd_running) {
    cd_remaining_ms = cd_duration_ms;
  } else {
    cd_remaining_ms += add_ms;
  }
  if (cd_time_label) {
    int min_val = (cd_remaining_ms / 60000) % 60;
    int sec = (cd_remaining_ms / 1000) % 60;
    char cbuf[32];
    snprintf(cbuf, sizeof(cbuf), "%02d:%02d", min_val, sec);
    lv_label_set_text(cd_time_label, cbuf);
  }
}

static void cd_btn_start_cb(lv_event_t * e) {
  play_beep(cd_running ? 500 : 1000, 80);
  if (cd_running) {
    cd_running = false;
    lv_label_set_text(cd_lbl_start, LV_SYMBOL_PLAY " Start");
  } else {
    if (cd_remaining_ms > 0) {
      cd_running = true;
      cd_start_time = millis();
      lv_label_set_text(cd_lbl_start, LV_SYMBOL_PAUSE " Pause");
    }
  }
}

static void cd_btn_reset_cb(lv_event_t * e) {
  play_beep(400, 100);
  cd_running = false;
  cd_duration_ms = 60000;
  cd_remaining_ms = 60000;
  if (cd_lbl_start) lv_label_set_text(cd_lbl_start, LV_SYMBOL_PLAY " Start");
}

// --- Alarm Controls ---
static void alarm_sw_cb(lv_event_t * e) {
  play_beep(700, 50);
  lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
  alarm_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
  prefs.putBool("alarm_en", alarm_enabled);
}

static void alarm_btn_change_cb(lv_event_t * e) {
  play_beep(800, 50);
  alarm_min += 15;
  if (alarm_min >= 60) {
    alarm_min = 0;
    alarm_hour = (alarm_hour + 1) % 24;
  }
  prefs.putInt("alarm_h", alarm_hour);
  prefs.putInt("alarm_m", alarm_min);

  if (alarm_lbl_time) {
    char abuf[32];
    int h12 = alarm_hour % 12;
    if (h12 == 0) h12 = 12;
    snprintf(abuf, sizeof(abuf), "%02d:%02d %s", h12, alarm_min, alarm_hour >= 12 ? "PM" : "AM");
    lv_label_set_text(alarm_lbl_time, abuf);
  }
}

static void btn_scan_event_cb(lv_event_t * e) {
  play_beep(850, 60);
  start_wifi_scan();
}

static char scanned_ssids[20][64];
static bool scanned_secure[20];

static void wifi_item_event_cb(lv_event_t * e) {
  play_beep(800, 50);
  uint32_t idx = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
  if (idx >= 20) return;

  strncpy(selected_ssid, scanned_ssids[idx], sizeof(selected_ssid) - 1);
  selected_ssid[sizeof(selected_ssid) - 1] = '\0';

  // If network is open (unencrypted), connect directly without password prompt
  if (!scanned_secure[idx]) {
    connect_to_wifi(selected_ssid, "");
    return;
  }

  if (pwd_modal != NULL) {
    lv_obj_delete_async(pwd_modal);
    pwd_modal = NULL;
    pwd_ta = NULL;
    keyboard = NULL;
  }

  pwd_modal = lv_obj_create(lv_scr_act());
  lv_obj_set_size(pwd_modal, SAFE_MODAL_W, 380);
  lv_obj_center(pwd_modal);
  lv_obj_set_style_bg_color(pwd_modal, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(pwd_modal, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_border_width(pwd_modal, 2, 0);
  lv_obj_set_style_radius(pwd_modal, 20, 0);
  lv_obj_remove_flag(pwd_modal, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * title = lv_label_create(pwd_modal);
  char title_buf[80];
  snprintf(title_buf, sizeof(title_buf), "Connect to: %s", selected_ssid);
  lv_label_set_text(title, title_buf);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0xCDD6F4), 0);

  pwd_ta = lv_textarea_create(pwd_modal);
  lv_textarea_set_password_mode(pwd_ta, false); // Visible by default on small watch screen to prevent typos
  lv_textarea_set_one_line(pwd_ta, true);
  lv_textarea_set_placeholder_text(pwd_ta, "Enter Wi-Fi Password...");
  lv_obj_set_size(pwd_ta, 312, 42);
  lv_obj_align(pwd_ta, LV_ALIGN_TOP_MID, 0, 30);

  // Pre-fill saved password if this SSID matches saved credentials
  String saved_ssid = prefs.getString("wifi_ssid", "");
  if (saved_ssid.equals(selected_ssid)) {
    String saved_pass = prefs.getString("wifi_pass", "");
    if (saved_pass.length() > 0) {
      lv_textarea_set_text(pwd_ta, saved_pass.c_str());
    }
  }

  keyboard = lv_keyboard_create(pwd_modal);
  lv_obj_set_size(keyboard, 316, 196);
  lv_obj_align(keyboard, LV_ALIGN_TOP_MID, 0, 78);
  lv_keyboard_set_textarea(keyboard, pwd_ta);

  lv_obj_t * btn_conn = lv_button_create(pwd_modal);
  lv_obj_set_size(btn_conn, 140, 38);
  lv_obj_align(btn_conn, LV_ALIGN_BOTTOM_LEFT, 8, -4);
  lv_obj_set_style_bg_color(btn_conn, lv_color_hex(0xA6E3A1), 0);
  lv_obj_t * lbl_conn = lv_label_create(btn_conn);
  lv_label_set_text(lbl_conn, LV_SYMBOL_WIFI " Connect");
  lv_obj_set_style_text_color(lbl_conn, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_conn);

  lv_obj_add_event_cb(btn_conn, [](lv_event_t * e) {
    play_beep(950, 80);
    char pass_buf[64] = "";
    if (pwd_ta) {
      const char * pass = lv_textarea_get_text(pwd_ta);
      if (pass) {
        strncpy(pass_buf, pass, sizeof(pass_buf) - 1);
        pass_buf[sizeof(pass_buf) - 1] = '\0';
      }
    }
    if (pwd_modal) {
      lv_obj_delete_async(pwd_modal);
      pwd_modal = NULL;
      pwd_ta = NULL;
      keyboard = NULL;
    }
    connect_to_wifi(selected_ssid, pass_buf);
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_cancel = lv_button_create(pwd_modal);
  lv_obj_set_size(btn_cancel, 140, 38);
  lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_RIGHT, -8, -4);
  lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0xF38BA8), 0);
  lv_obj_t * lbl_cancel = lv_label_create(btn_cancel);
  lv_label_set_text(lbl_cancel, LV_SYMBOL_CLOSE " Cancel");
  lv_obj_set_style_text_color(lbl_cancel, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_cancel);

  lv_obj_add_event_cb(btn_cancel, [](lv_event_t * e) {
    play_beep(450, 60);
    if (pwd_modal) {
      lv_obj_delete_async(pwd_modal);
      pwd_modal = NULL;
      pwd_ta = NULL;
      keyboard = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);
}

// --- Asynchronous Wi-Fi Scanning ---
void start_wifi_scan() {
  if (!wifi_list || !wifi_power_on || wifi_scan_in_progress) return;

  int scan_status = WiFi.scanComplete();
  if (scan_status == WIFI_SCAN_RUNNING) return;

  if (wifi_connect_in_progress) {
    wifi_connect_in_progress = false;
    WiFi.disconnect(false);
  }

  lv_obj_clean(wifi_list);

  lv_obj_t * lbl_loading = lv_label_create(wifi_list);
  lv_label_set_text(lbl_loading, "Scanning Wi-Fi in background...");
  lv_obj_align(lbl_loading, LV_ALIGN_CENTER, 0, 20);

  if (wifi_page_status_lbl) {
    lv_label_set_text(wifi_page_status_lbl, "Status: Scanning 2.4GHz networks...");
    lv_obj_set_style_text_color(wifi_page_status_lbl, lv_color_hex(0xF9E2AF), 0);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.setSleep(true);
  WiFi.scanDelete();
  WiFi.scanNetworks(true);
  wifi_scan_in_progress = true;
  USBSerial.println("Asynchronous Wi-Fi scan started...");
}

void update_wifi_scan_results() {
  if (!wifi_scan_in_progress) return;

  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;

  wifi_scan_in_progress = false;

  if (n < 0) {
    USBSerial.printf("Wi-Fi scan returned error: %d\n", n);
    if (wifi_page_status_lbl) {
      lv_label_set_text(wifi_page_status_lbl, "Scan failed. Tap Scan to retry.");
      lv_obj_set_style_text_color(wifi_page_status_lbl, lv_color_hex(0xF38BA8), 0);
    }
    return;
  }

  USBSerial.printf("Wi-Fi scan finished. %d networks found\n", n);

  if (wifi_page_status_lbl) {
    char sbuf[64];
    snprintf(sbuf, sizeof(sbuf), "Found %d network(s). Tap to connect:", n);
    lv_label_set_text(wifi_page_status_lbl, sbuf);
    lv_obj_set_style_text_color(wifi_page_status_lbl, lv_color_hex(0xA6E3A1), 0);
  }

  if (!wifi_list) {
    WiFi.scanDelete();
    return;
  }
  lv_obj_clean(wifi_list);

  if (n == 0) {
    lv_obj_t * lbl_none = lv_label_create(wifi_list);
    lv_label_set_text(lbl_none, "No 2.4GHz networks found.");
    lv_obj_align(lbl_none, LV_ALIGN_CENTER, 0, 20);
  } else {
    int max_items = n < 20 ? n : 20;
    for (int i = 0; i < max_items; ++i) {
      String ssid = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      bool is_secure = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);

      strncpy(scanned_ssids[i], ssid.c_str(), sizeof(scanned_ssids[i]) - 1);
      scanned_ssids[i][sizeof(scanned_ssids[i]) - 1] = '\0';
      scanned_secure[i] = is_secure;

      char btn_txt[128];
      snprintf(btn_txt, sizeof(btn_txt), "%s (%ld dBm) %s",
               scanned_ssids[i], (long)rssi, is_secure ? "[SEC]" : "[OPEN]");

      lv_obj_t * btn = lv_list_add_button(wifi_list, LV_SYMBOL_WIFI, btn_txt);
      lv_obj_add_event_cb(btn, wifi_item_event_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)i);
    }
  }

  WiFi.scanDelete();
}

void connect_to_wifi(const char* ssid, const char* password) {
  if (!wifi_power_on || !ssid || strlen(ssid) == 0) return;

  char ssid_buf[64];
  char pass_buf[64];
  strncpy(ssid_buf, ssid, sizeof(ssid_buf) - 1);
  ssid_buf[sizeof(ssid_buf) - 1] = '\0';
  if (password) {
    strncpy(pass_buf, password, sizeof(pass_buf) - 1);
    pass_buf[sizeof(pass_buf) - 1] = '\0';
  } else {
    pass_buf[0] = '\0';
  }

  USBSerial.printf("Connecting to Wi-Fi SSID: '%s'...\n", ssid_buf);

  if (wifi_status_label) {
    char buf[80];
    snprintf(buf, sizeof(buf), "Wi-Fi: Connecting to %s...", ssid_buf);
    lv_label_set_text(wifi_status_label, buf);
  }
  if (wifi_page_status_lbl) {
    char pbuf[80];
    snprintf(pbuf, sizeof(pbuf), "Connecting to %s...", ssid_buf);
    lv_label_set_text(wifi_page_status_lbl, pbuf);
    lv_obj_set_style_text_color(wifi_page_status_lbl, lv_color_hex(0xF9E2AF), 0);
  }

  // Stop any active scan before starting connection
  if (wifi_scan_in_progress || WiFi.scanComplete() == WIFI_SCAN_RUNNING) {
    WiFi.scanDelete();
    wifi_scan_in_progress = false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.setSleep(true);
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

  if (strlen(pass_buf) == 0) {
    WiFi.setMinSecurity(WIFI_AUTH_OPEN);
    WiFi.begin(ssid_buf, NULL);
  } else {
    WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
    WiFi.begin(ssid_buf, pass_buf);
  }

  wifi_connect_in_progress = true;
  wifi_connect_start_ms = millis();

  prefs.putString("wifi_ssid", ssid_buf);
  prefs.putString("wifi_pass", pass_buf);
}

// --- FACE 1: SUNDIAL SIMULATOR WIDGET RENDERER ---
void render_sundial(int total_minutes) {
  if (!sundial_card || !sun_obj || !shadow_obj) return;

  bool is_daytime = (total_minutes >= 360 && total_minutes <= 1080);

  lv_obj_set_style_bg_color(sundial_card, is_daytime ? lv_color_hex(0x181825) : lv_color_hex(0x0B0B12), 0);

  float angle_deg = 180.0f - ((float)(total_minutes - 360) / 720.0f) * 180.0f;
  float rad = angle_deg * (M_PI / 180.0f);

  int cx = 166;
  int cy = 135;
  int radius_x = 120;
  int radius_y = 78;

  int body_x = cx + (int)(cosf(rad) * radius_x);
  int body_y = cy - (int)(sinf(rad) * radius_y);

  lv_obj_align(sun_obj, LV_ALIGN_TOP_LEFT, body_x - 15, body_y - 15);

  if (is_daytime) {
    lv_obj_set_style_bg_color(sun_obj, lv_color_hex(0xF9E2AF), 0);
    lv_obj_set_style_border_color(sun_obj, lv_color_hex(0xFAB387), 0);
  } else {
    lv_obj_set_style_bg_color(sun_obj, lv_color_hex(0xCDD6F4), 0);
    lv_obj_set_style_border_color(sun_obj, lv_color_hex(0x89B4FA), 0);
  }

  if (is_daytime) {
    float shadow_rad = rad + M_PI;
    int shadow_len = (int)(50.0f * (1.0f - 0.4f * sinf(rad)));
    int shadow_x = cx + (int)(cosf(shadow_rad) * shadow_len);
    int shadow_y = cy - (int)(sinf(shadow_rad) * shadow_len * 0.4f);

    lv_obj_clear_flag(shadow_obj, LV_OBJ_FLAG_HIDDEN);
    int shadow_w = (shadow_x > cx) ? (shadow_x - cx + 6) : (cx - shadow_x + 6);
    int shadow_left = (shadow_x < cx) ? shadow_x : cx;

    lv_obj_set_size(shadow_obj, shadow_w, 6);
    lv_obj_align(shadow_obj, LV_ALIGN_TOP_LEFT, shadow_left, shadow_y - 3);
  } else {
    lv_obj_add_flag(shadow_obj, LV_OBJ_FLAG_HIDDEN);
  }

  if (sundial_info_lbl) {
    int h = total_minutes / 60;
    int m = total_minutes % 60;
    int h12 = h % 12;
    if (h12 == 0) h12 = 12;
    const char* ampm = (h >= 12) ? "PM" : "AM";

    char sbuf[96];
    if (is_daytime) {
      int solar_angle = (int)(angle_deg);
      snprintf(sbuf, sizeof(sbuf), "Solar Time: %02d:%02d %s  |  Sun: %d°",
               h12, m, ampm, solar_angle);
    } else {
      snprintf(sbuf, sizeof(sbuf), "Night Time: %02d:%02d %s  |  Moonlit",
               h12, m, ampm);
    }
    lv_label_set_text(sundial_info_lbl, sbuf);
  }
}

static void sundial_slider_event_cb(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  sundial_sim_min = lv_slider_get_value(slider);
  render_sundial(sundial_sim_min);
}

static void sundial_mode_btn_cb(lv_event_t * e) {
  play_beep(750, 60);
  sundial_manual_mode = !sundial_manual_mode;
  if (sundial_manual_mode) {
    lv_label_set_text(sundial_mode_lbl, "Mode: Scrub");
    lv_obj_clear_flag(sundial_slider, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_label_set_text(sundial_mode_lbl, "Mode: Live Clock");
    lv_obj_add_flag(sundial_slider, LV_OBJ_FLAG_HIDDEN);
    update_sundial_display();
  }
}

void update_sundial_display() {
  if (active_page_idx != 0 || selected_face_idx != 1) return;

  if (!sundial_manual_mode) {
    int yr, mo, dy, hour = 12, minute = 0, sec;
    get_watch_datetime(yr, mo, dy, hour, minute, sec);
    sundial_sim_min = hour * 60 + minute;
    if (sundial_slider) lv_slider_set_value(sundial_slider, sundial_sim_min, LV_ANIM_OFF);
  }
  render_sundial(sundial_sim_min);
}

// --- FACE 2: NEON CYBERPUNK DISPLAY UPDATE ---
void update_neon_display(int hour, int minute, int sec, int year, int month, int day) {
  if (!neon_time_lbl || !neon_date_lbl || !neon_sec_bar) return;

  char nbuf[32];
  if (use_12h_format) {
    int h12 = hour % 12;
    if (h12 == 0) h12 = 12;
    snprintf(nbuf, sizeof(nbuf), "%02d:%02d:%02d", h12, minute, sec);
  } else {
    snprintf(nbuf, sizeof(nbuf), "%02d:%02d:%02d", hour, minute, sec);
  }
  lv_label_set_text(neon_time_lbl, nbuf);

  char dbuf[64];
  snprintf(dbuf, sizeof(dbuf), "CYBER DATE: %04d.%02d.%02d", year, month, day);
  lv_label_set_text(neon_date_lbl, dbuf);

  int pct = (sec * 100) / 59;
  lv_bar_set_value(neon_sec_bar, pct, LV_ANIM_OFF);
}

// --- FACE 3: ANALOG CLASSIC DISPLAY UPDATE ---
void update_analog_display(int hour, int minute, int sec) {
  if (!analog_hour_hand || !analog_min_hand || !analog_sec_hand) return;

  int cx = 152;
  int cy = 152;

  // Hour Hand (Length 60px)
  float h_angle = ((hour % 12) * 30.0f + minute * 0.5f) * (M_PI / 180.0f);
  int h_len = 60;
  int hx = cx + (int)(sinf(h_angle) * h_len);
  int hy = cy - (int)(cosf(h_angle) * h_len);

  static lv_point_precise_t hour_pts[2];
  hour_pts[0].x = cx; hour_pts[0].y = cy;
  hour_pts[1].x = hx; hour_pts[1].y = hy;
  lv_line_set_points(analog_hour_hand, hour_pts, 2);

  // Minute Hand (Length 96px)
  float m_angle = (minute * 6.0f + sec * 0.1f) * (M_PI / 180.0f);
  int m_len = 96;
  int mx = cx + (int)(sinf(m_angle) * m_len);
  int my = cy - (int)(cosf(m_angle) * m_len);

  static lv_point_precise_t min_pts[2];
  min_pts[0].x = cx; min_pts[0].y = cy;
  min_pts[1].x = mx; min_pts[1].y = my;
  lv_line_set_points(analog_min_hand, min_pts, 2);

  // Second Hand (Length 114px)
  float s_angle = (sec * 6.0f) * (M_PI / 180.0f);
  int s_len = 114;
  int sx = cx + (int)(sinf(s_angle) * s_len);
  int sy = cy - (int)(cosf(s_angle) * s_len);

  static lv_point_precise_t sec_pts[2];
  sec_pts[0].x = cx; sec_pts[0].y = cy;
  sec_pts[1].x = sx; sec_pts[1].y = sy;
  lv_line_set_points(analog_sec_hand, sec_pts, 2);
}

// --- FACE 4: ASCII MATRIX TERMINAL DISPLAY UPDATE ---
static const char* ascii_art_digits[11][5] = {
  { "███", "█ █", "█ █", "█ █", "███" }, // 0
  { " █ ", "██ ", " █ ", " █ ", "███" }, // 1
  { "███", "  █", "███", "█  ", "███" }, // 2
  { "███", "  █", "███", "  █", "███" }, // 3
  { "█ █", "█ █", "███", "  █", "  █" }, // 4
  { "███", "█  ", "███", "  █", "███" }, // 5
  { "███", "█  ", "███", "█ █", "███" }, // 6
  { "███", "  █", "  █", "  █", "  █" }, // 7
  { "███", "█ █", "███", "█ █", "███" }, // 8
  { "███", "█ █", "███", "  █", "███" }, // 9
  { "   ", " █ ", "   ", " █ ", "   " }  // :
};

void update_ascii_display(int hour, int minute, int sec, int year, int month, int day) {
  if (!ascii_sub_info || !ascii_lines[0]) return;

  int display_h = hour;
  const char* ampm_str = "";
  if (use_12h_format) {
    display_h = hour % 12;
    if (display_h == 0) display_h = 12;
    ampm_str = (hour >= 12) ? "PM" : "AM";
  }

  int d0 = display_h / 10;
  int d1 = display_h % 10;
  int d2 = minute / 10;
  int d3 = minute % 10;

  for (int row = 0; row < 5; row++) {
    char row_buf[64];
    if (d0 == 0) {
      snprintf(row_buf, sizeof(row_buf), "   %s  %s  %s  %s",
               ascii_art_digits[d1][row],
               ascii_art_digits[10][row],
               ascii_art_digits[d2][row],
               ascii_art_digits[d3][row]);
    } else {
      snprintf(row_buf, sizeof(row_buf), "%s  %s  %s  %s  %s",
               ascii_art_digits[d0][row],
               ascii_art_digits[d1][row],
               ascii_art_digits[10][row],
               ascii_art_digits[d2][row],
               ascii_art_digits[d3][row]);
    }
    if (ascii_lines[row]) lv_label_set_text(ascii_lines[row], row_buf);
  }

  char sub_buf[96];
  snprintf(sub_buf, sizeof(sub_buf), "SEC:%02d  |  %04d.%02d.%02d  |  %s",
           sec, year, month, day, ampm_str);
  lv_label_set_text(ascii_sub_info, sub_buf);
}

// --- Create Modern Inset Watch UI (Safe-Area Fitted for 2.06" Rounded AMOLED) ---
void create_ui() {
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x11111B), 0);

  // --- TOP STATUS BAR WITH 4 CLICKABLE NAV PILLS (Clock, Tools, BT, Settings) ---
  // Positioned at y=SAFE_TOP_Y (18) with width SAFE_BAR_W (336) to clear the 72px corner bezel curve
  top_bar = lv_obj_create(lv_scr_act());
  lv_obj_set_size(top_bar, SAFE_BAR_W, SAFE_BAR_H);
  lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, SAFE_TOP_Y);
  lv_obj_set_style_bg_color(top_bar, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(top_bar, lv_color_hex(0x313244), 0);
  lv_obj_set_style_border_width(top_bar, 1, 0);
  lv_obj_set_style_radius(top_bar, 20, 0);
  lv_obj_set_style_pad_all(top_bar, 2, 0);
  lv_obj_remove_flag(top_bar, LV_OBJ_FLAG_SCROLLABLE);

  // Nav Pill 0: Clock
  bar_btn_clock = lv_button_create(top_bar);
  lv_obj_set_size(bar_btn_clock, 44, 30);
  lv_obj_align(bar_btn_clock, LV_ALIGN_LEFT_MID, 4, 0);
  lv_obj_set_style_bg_color(bar_btn_clock, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_radius(bar_btn_clock, 15, 0);
  lv_obj_t * lbl_c = lv_label_create(bar_btn_clock);
  lv_label_set_text(lbl_c, "CLK");
  lv_obj_set_style_text_color(lbl_c, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_c);
  lv_obj_add_event_cb(bar_btn_clock, nav_to_page_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)0);

  // Nav Pill 1: Tools
  bar_btn_tools = lv_button_create(top_bar);
  lv_obj_set_size(bar_btn_tools, 42, 30);
  lv_obj_align(bar_btn_tools, LV_ALIGN_LEFT_MID, 52, 0);
  lv_obj_set_style_bg_color(bar_btn_tools, lv_color_hex(0x313244), 0);
  lv_obj_set_style_radius(bar_btn_tools, 15, 0);
  lv_obj_t * lbl_t = lv_label_create(bar_btn_tools);
  lv_label_set_text(lbl_t, LV_SYMBOL_BELL);
  lv_obj_center(lbl_t);
  lv_obj_add_event_cb(bar_btn_tools, nav_to_page_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)1);

  // Nav Pill 2: Bluetooth
  bar_btn_bt = lv_button_create(top_bar);
  lv_obj_set_size(bar_btn_bt, 42, 30);
  lv_obj_align(bar_btn_bt, LV_ALIGN_LEFT_MID, 98, 0);
  lv_obj_set_style_bg_color(bar_btn_bt, lv_color_hex(0x313244), 0);
  lv_obj_set_style_radius(bar_btn_bt, 15, 0);
  bar_bt_lbl = lv_label_create(bar_btn_bt);
  lv_label_set_text(bar_bt_lbl, LV_SYMBOL_BLUETOOTH);
  lv_obj_center(bar_bt_lbl);
  lv_obj_add_event_cb(bar_btn_bt, nav_to_page_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)2);

  // Nav Pill 3: Settings (Wi-Fi menu removed)
  bar_btn_settings = lv_button_create(top_bar);
  lv_obj_set_size(bar_btn_settings, 42, 30);
  lv_obj_align(bar_btn_settings, LV_ALIGN_LEFT_MID, 144, 0);
  lv_obj_set_style_bg_color(bar_btn_settings, lv_color_hex(0x313244), 0);
  lv_obj_set_style_radius(bar_btn_settings, 15, 0);
  lv_obj_t * lbl_set = lv_label_create(bar_btn_settings);
  lv_label_set_text(lbl_set, LV_SYMBOL_SETTINGS);
  lv_obj_center(lbl_set);
  lv_obj_add_event_cb(bar_btn_settings, nav_to_page_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)3);

  // Status Labels (Time & Battery)
  bar_time_lbl = lv_label_create(top_bar);
  lv_label_set_text(bar_time_lbl, "12:00");
  lv_obj_set_style_text_font(bar_time_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bar_time_lbl, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(bar_time_lbl, LV_ALIGN_RIGHT_MID, -48, 0);

  bar_batt_lbl = lv_label_create(top_bar);
  lv_label_set_text(bar_batt_lbl, "100%");
  lv_obj_set_style_text_font(bar_batt_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bar_batt_lbl, lv_color_hex(0xA6E3A1), 0);
  lv_obj_align(bar_batt_lbl, LV_ALIGN_RIGHT_MID, -6, 0);

  // ================= PAGE 0: MAIN WATCH FACES CONTAINER =================
  page_clock = lv_obj_create(lv_scr_act());
  lv_obj_set_size(page_clock, SAFE_PAGE_W, SAFE_PAGE_H);
  lv_obj_align(page_clock, LV_ALIGN_TOP_MID, 0, SAFE_PAGE_Y);
  lv_obj_set_style_bg_color(page_clock, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(page_clock, 0, 0);
  lv_obj_set_style_pad_all(page_clock, 0, 0);
  lv_obj_remove_flag(page_clock, LV_OBJ_FLAG_SCROLLABLE);

  // Top Quick Controls Bar (Watch Faces, Manual Set Time, Sleep, Sync)
  lv_obj_t * quick_bar = lv_obj_create(page_clock);
  lv_obj_set_size(quick_bar, SAFE_CARD_W, 42);
  lv_obj_align(quick_bar, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(quick_bar, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_width(quick_bar, 0, 0);
  lv_obj_set_style_radius(quick_bar, 21, 0);
  lv_obj_set_style_pad_all(quick_bar, 3, 0);
  lv_obj_remove_flag(quick_bar, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * btn_gallery = lv_button_create(quick_bar);
  lv_obj_set_size(btn_gallery, 114, 32);
  lv_obj_align(btn_gallery, LV_ALIGN_LEFT_MID, 2, 0);
  lv_obj_set_style_bg_color(btn_gallery, lv_color_hex(0xCBA6F7), 0);
  lv_obj_set_style_radius(btn_gallery, 16, 0);
  lv_obj_t * lbl_gal = lv_label_create(btn_gallery);
  lv_label_set_text(lbl_gal, LV_SYMBOL_IMAGE " Faces");
  lv_obj_set_style_text_font(lbl_gal, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_gal, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_gal);
  lv_obj_add_event_cb(btn_gallery, [](lv_event_t * e) {
    open_watch_face_gallery();
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_set_clock = lv_button_create(quick_bar);
  lv_obj_set_size(btn_set_clock, 70, 32);
  lv_obj_align(btn_set_clock, LV_ALIGN_LEFT_MID, 120, 0);
  lv_obj_set_style_bg_color(btn_set_clock, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_radius(btn_set_clock, 16, 0);
  lv_obj_t * lbl_set_clk = lv_label_create(btn_set_clock);
  lv_label_set_text(lbl_set_clk, LV_SYMBOL_EDIT " Set");
  lv_obj_set_style_text_font(lbl_set_clk, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_set_clk, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_set_clk);
  lv_obj_add_event_cb(btn_set_clock, [](lv_event_t * e) {
    open_manual_time_modal();
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_sleep = lv_button_create(quick_bar);
  lv_obj_set_size(btn_sleep, 62, 32);
  lv_obj_align(btn_sleep, LV_ALIGN_RIGHT_MID, -66, 0);
  lv_obj_set_style_bg_color(btn_sleep, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_radius(btn_sleep, 16, 0);
  lv_obj_t * lbl_sleep = lv_label_create(btn_sleep);
  lv_label_set_text(lbl_sleep, LV_SYMBOL_POWER);
  lv_obj_center(lbl_sleep);
  lv_obj_add_event_cb(btn_sleep, btn_sleep_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_sync = lv_button_create(quick_bar);
  lv_obj_set_size(btn_sync, 60, 32);
  lv_obj_align(btn_sync, LV_ALIGN_RIGHT_MID, -2, 0);
  lv_obj_set_style_bg_color(btn_sync, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_radius(btn_sync, 16, 0);
  lv_obj_t * lbl_sync = lv_label_create(btn_sync);
  lv_label_set_text(lbl_sync, LV_SYMBOL_BLUETOOTH);
  lv_obj_center(lbl_sync);
  lv_obj_add_event_cb(btn_sync, btn_sync_event_cb, LV_EVENT_CLICKED, NULL);

  // ---------------- WATCH FACE 0: MODERN DIGITAL ----------------
  face_container_modern = lv_obj_create(page_clock);
  lv_obj_set_size(face_container_modern, SAFE_FACE_W, SAFE_FACE_H);
  lv_obj_align(face_container_modern, LV_ALIGN_TOP_MID, 0, 46);
  lv_obj_set_style_bg_color(face_container_modern, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(face_container_modern, 0, 0);
  lv_obj_set_style_pad_all(face_container_modern, 4, 0);
  lv_obj_set_style_pad_row(face_container_modern, 6, 0);
  lv_obj_set_flex_flow(face_container_modern, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(face_container_modern, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(face_container_modern, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * clock_card = lv_obj_create(face_container_modern);
  lv_obj_set_size(clock_card, 332, 132);
  lv_obj_set_style_bg_color(clock_card, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(clock_card, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_border_width(clock_card, 2, 0);
  lv_obj_set_style_radius(clock_card, 22, 0);
  lv_obj_remove_flag(clock_card, LV_OBJ_FLAG_SCROLLABLE);

  clock_time_label = lv_label_create(clock_card);
  lv_label_set_text(clock_time_label, "--:--:--");
  lv_obj_set_style_text_font(clock_time_label, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(clock_time_label, lv_color_hex(0x89B4FA), 0);
  lv_obj_align(clock_time_label, LV_ALIGN_CENTER, -22, -14);

  clock_ampm_label = lv_label_create(clock_card);
  lv_label_set_text(clock_ampm_label, "AM");
  lv_obj_set_style_text_font(clock_ampm_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(clock_ampm_label, lv_color_hex(0xF9E2AF), 0);
  lv_obj_align(clock_ampm_label, LV_ALIGN_CENTER, 112, -10);

  clock_date_label = lv_label_create(clock_card);
  lv_label_set_text(clock_date_label, "Initializing Date...");
  lv_obj_set_style_text_font(clock_date_label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(clock_date_label, lv_color_hex(0xA6ADC8), 0);
  lv_obj_align(clock_date_label, LV_ALIGN_CENTER, 0, 32);

  tz_dropdown = lv_dropdown_create(face_container_modern);
  lv_dropdown_clear_options(tz_dropdown);
  for (int i = 0; i < num_tz_options; i++) {
    lv_dropdown_add_option(tz_dropdown, tz_options[i].name, i);
  }
  lv_dropdown_set_selected(tz_dropdown, current_tz_idx);
  lv_obj_set_size(tz_dropdown, 332, 38);
  lv_obj_set_style_bg_color(tz_dropdown, lv_color_hex(0x181825), 0);
  lv_obj_set_style_text_color(tz_dropdown, lv_color_hex(0xCDD6F4), 0);
  lv_obj_set_style_radius(tz_dropdown, 14, 0);
  lv_obj_add_event_cb(tz_dropdown, tz_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t * info_card = lv_obj_create(face_container_modern);
  lv_obj_set_size(info_card, 332, 124);
  lv_obj_set_style_bg_color(info_card, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(info_card, lv_color_hex(0x313244), 0);
  lv_obj_set_style_radius(info_card, 20, 0);
  lv_obj_set_style_pad_all(info_card, 8, 0);
  lv_obj_set_style_pad_row(info_card, 3, 0);
  lv_obj_set_flex_flow(info_card, LV_FLEX_FLOW_COLUMN);
  lv_obj_remove_flag(info_card, LV_OBJ_FLAG_SCROLLABLE);

  bt_clock_status_label = lv_label_create(info_card);
  lv_label_set_text(bt_clock_status_label, "BLE Ready (" BLE_DEVICE_NAME ")");
  lv_obj_set_style_text_color(bt_clock_status_label, lv_color_hex(0xCBA6F7), 0);

  wifi_status_label = lv_label_create(info_card);
  lv_label_set_text(wifi_status_label, last_android_req_str);
  lv_obj_set_style_text_color(wifi_status_label, lv_color_hex(0x89B4FA), 0);

  batt_status_label = lv_label_create(info_card);
  lv_label_set_text(batt_status_label, "Battery: Monitoring...");
  lv_obj_set_style_text_color(batt_status_label, lv_color_hex(0xA6E3A1), 0);

  ntp_status_label = lv_label_create(info_card);
  lv_label_set_text(ntp_status_label, "Sync: Android BLE or 'Set'");
  lv_obj_set_style_text_color(ntp_status_label, lv_color_hex(0xF9E2AF), 0);

  // ---------------- WATCH FACE 1: SUNDIAL SIMULATOR ----------------
  face_container_sundial = lv_obj_create(page_clock);
  lv_obj_set_size(face_container_sundial, SAFE_FACE_W, SAFE_FACE_H);
  lv_obj_align(face_container_sundial, LV_ALIGN_TOP_MID, 0, 46);
  lv_obj_set_style_bg_color(face_container_sundial, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(face_container_sundial, 0, 0);

  sundial_card = lv_obj_create(face_container_sundial);
  lv_obj_set_size(sundial_card, 332, 210);
  lv_obj_align(sundial_card, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(sundial_card, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(sundial_card, lv_color_hex(0xF9E2AF), 0);
  lv_obj_set_style_border_width(sundial_card, 2, 0);
  lv_obj_set_style_radius(sundial_card, 22, 0);
  lv_obj_remove_flag(sundial_card, LV_OBJ_FLAG_SCROLLABLE);

  sundial_plate = lv_obj_create(sundial_card);
  lv_obj_set_size(sundial_plate, 120, 120);
  lv_obj_align(sundial_plate, LV_ALIGN_CENTER, 0, 22);
  lv_obj_set_style_bg_color(sundial_plate, lv_color_hex(0x313244), 0);
  lv_obj_set_style_border_color(sundial_plate, lv_color_hex(0xF9E2AF), 0);
  lv_obj_set_style_border_width(sundial_plate, 3, 0);
  lv_obj_set_style_radius(sundial_plate, 60, 0);

  shadow_obj = lv_obj_create(sundial_card);
  lv_obj_set_size(shadow_obj, 50, 6);
  lv_obj_set_style_bg_color(shadow_obj, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(shadow_obj, 0, 0);
  lv_obj_set_style_radius(shadow_obj, 3, 0);

  gnomon_obj = lv_obj_create(sundial_card);
  lv_obj_set_size(gnomon_obj, 12, 12);
  lv_obj_align(gnomon_obj, LV_ALIGN_CENTER, 0, 22);
  lv_obj_set_style_bg_color(gnomon_obj, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_border_width(gnomon_obj, 0, 0);
  lv_obj_set_style_radius(gnomon_obj, 6, 0);

  sun_obj = lv_obj_create(sundial_card);
  lv_obj_set_size(sun_obj, 30, 30);
  lv_obj_set_style_bg_color(sun_obj, lv_color_hex(0xF9E2AF), 0);
  lv_obj_set_style_border_color(sun_obj, lv_color_hex(0xFAB387), 0);
  lv_obj_set_style_border_width(sun_obj, 2, 0);
  lv_obj_set_style_radius(sun_obj, 15, 0);

  sundial_info_lbl = lv_label_create(face_container_sundial);
  lv_label_set_text(sundial_info_lbl, "Solar Time: --:--");
  lv_obj_set_style_text_font(sundial_info_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(sundial_info_lbl, lv_color_hex(0xF9E2AF), 0);
  lv_obj_align(sundial_info_lbl, LV_ALIGN_TOP_MID, 0, 218);

  sundial_mode_btn = lv_button_create(face_container_sundial);
  lv_obj_set_size(sundial_mode_btn, 166, 34);
  lv_obj_align(sundial_mode_btn, LV_ALIGN_TOP_MID, 0, 244);
  lv_obj_set_style_bg_color(sundial_mode_btn, lv_color_hex(0x313244), 0);
  lv_obj_set_style_radius(sundial_mode_btn, 17, 0);
  sundial_mode_lbl = lv_label_create(sundial_mode_btn);
  lv_label_set_text(sundial_mode_lbl, "Mode: Live Clock");
  lv_obj_center(sundial_mode_lbl);
  lv_obj_add_event_cb(sundial_mode_btn, sundial_mode_btn_cb, LV_EVENT_CLICKED, NULL);

  sundial_slider = lv_slider_create(face_container_sundial);
  lv_slider_set_range(sundial_slider, 0, 1439);
  lv_slider_set_value(sundial_slider, 720, LV_ANIM_OFF);
  lv_obj_set_size(sundial_slider, 296, 18);
  lv_obj_align(sundial_slider, LV_ALIGN_TOP_MID, 0, 288);
  lv_obj_set_style_bg_color(sundial_slider, lv_color_hex(0x313244), 0);
  lv_obj_set_style_bg_color(sundial_slider, lv_color_hex(0xF9E2AF), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(sundial_slider, lv_color_hex(0xCDD6F4), LV_PART_KNOB);
  lv_obj_add_flag(sundial_slider, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(sundial_slider, sundial_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // ---------------- WATCH FACE 2: NEON CYBERPUNK ----------------
  face_container_neon = lv_obj_create(page_clock);
  lv_obj_set_size(face_container_neon, SAFE_FACE_W, SAFE_FACE_H);
  lv_obj_align(face_container_neon, LV_ALIGN_TOP_MID, 0, 46);
  lv_obj_set_style_bg_color(face_container_neon, lv_color_hex(0x05050A), 0);
  lv_obj_set_style_border_color(face_container_neon, lv_color_hex(0x00E5FF), 0);
  lv_obj_set_style_border_width(face_container_neon, 2, 0);
  lv_obj_set_style_radius(face_container_neon, 24, 0);

  neon_time_lbl = lv_label_create(face_container_neon);
  lv_label_set_text(neon_time_lbl, "00:00:00");
  lv_obj_set_style_text_font(neon_time_lbl, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(neon_time_lbl, lv_color_hex(0x00E5FF), 0);
  lv_obj_align(neon_time_lbl, LV_ALIGN_CENTER, 0, -40);

  neon_date_lbl = lv_label_create(face_container_neon);
  lv_label_set_text(neon_date_lbl, "CYBER DATE: 2026.08.05");
  lv_obj_set_style_text_font(neon_date_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(neon_date_lbl, lv_color_hex(0xFF007F), 0);
  lv_obj_align(neon_date_lbl, LV_ALIGN_CENTER, 0, 18);

  neon_sec_bar = lv_bar_create(face_container_neon);
  lv_bar_set_range(neon_sec_bar, 0, 100);
  lv_obj_set_size(neon_sec_bar, 280, 16);
  lv_obj_align(neon_sec_bar, LV_ALIGN_CENTER, 0, 65);
  lv_obj_set_style_bg_color(neon_sec_bar, lv_color_hex(0x1A1A2E), 0);
  lv_obj_set_style_bg_color(neon_sec_bar, lv_color_hex(0x00E5FF), LV_PART_INDICATOR);
  lv_obj_set_style_radius(neon_sec_bar, 8, 0);

  // ---------------- WATCH FACE 3: ANALOG CLASSIC ----------------
  face_container_analog = lv_obj_create(page_clock);
  lv_obj_set_size(face_container_analog, SAFE_FACE_W, SAFE_FACE_H);
  lv_obj_align(face_container_analog, LV_ALIGN_TOP_MID, 0, 46);
  lv_obj_set_style_bg_color(face_container_analog, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(face_container_analog, 0, 0);

  analog_dial_bg = lv_obj_create(face_container_analog);
  lv_obj_set_size(analog_dial_bg, 304, 304);
  lv_obj_align(analog_dial_bg, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(analog_dial_bg, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(analog_dial_bg, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_border_width(analog_dial_bg, 3, 0);
  lv_obj_set_style_radius(analog_dial_bg, 152, 0);

  // Dial Hour Markers (12, 3, 6, 9)
  lv_obj_t * m12 = lv_label_create(analog_dial_bg);
  lv_label_set_text(m12, "12");
  lv_obj_set_style_text_font(m12, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(m12, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(m12, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t * m6 = lv_label_create(analog_dial_bg);
  lv_label_set_text(m6, "6");
  lv_obj_set_style_text_font(m6, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(m6, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(m6, LV_ALIGN_BOTTOM_MID, 0, -10);

  lv_obj_t * m3 = lv_label_create(analog_dial_bg);
  lv_label_set_text(m3, "3");
  lv_obj_set_style_text_font(m3, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(m3, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(m3, LV_ALIGN_RIGHT_MID, -15, 0);

  lv_obj_t * m9 = lv_label_create(analog_dial_bg);
  lv_label_set_text(m9, "9");
  lv_obj_set_style_text_font(m9, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(m9, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(m9, LV_ALIGN_LEFT_MID, 15, 0);

  // Analog Hands (Created as LVGL line objects for smooth rotated watch hands)
  analog_hour_hand = lv_line_create(analog_dial_bg);
  lv_obj_set_style_line_width(analog_hour_hand, 8, 0);
  lv_obj_set_style_line_color(analog_hour_hand, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_line_rounded(analog_hour_hand, true, 0);

  analog_min_hand = lv_line_create(analog_dial_bg);
  lv_obj_set_style_line_width(analog_min_hand, 5, 0);
  lv_obj_set_style_line_color(analog_min_hand, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_line_rounded(analog_min_hand, true, 0);

  analog_sec_hand = lv_line_create(analog_dial_bg);
  lv_obj_set_style_line_width(analog_sec_hand, 2, 0);
  lv_obj_set_style_line_color(analog_sec_hand, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_line_rounded(analog_sec_hand, true, 0);

  // Center Pin
  lv_obj_t * pin = lv_obj_create(analog_dial_bg);
  lv_obj_set_size(pin, 16, 16);
  lv_obj_align(pin, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(pin, lv_color_hex(0xF9E2AF), 0);
  lv_obj_set_style_border_color(pin, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(pin, 2, 0);
  lv_obj_set_style_radius(pin, 8, 0);

  // ================= PAGE 1: SMARTWATCH TOOLS =================
  page_tools = lv_obj_create(lv_scr_act());
  lv_obj_set_size(page_tools, SAFE_PAGE_W, SAFE_PAGE_H);
  lv_obj_align(page_tools, LV_ALIGN_TOP_MID, 0, SAFE_PAGE_Y);
  lv_obj_set_style_bg_color(page_tools, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(page_tools, 0, 0);
  lv_obj_set_style_pad_all(page_tools, 4, 0);
  lv_obj_set_style_pad_row(page_tools, 6, 0);
  lv_obj_set_flex_flow(page_tools, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(page_tools, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(page_tools, LV_OBJ_FLAG_SCROLLABLE);

  // SECTION 1: STOPWATCH
  lv_obj_t * sw_card = lv_obj_create(page_tools);
  lv_obj_set_size(sw_card, SAFE_CARD_W, 120);
  lv_obj_set_style_bg_color(sw_card, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(sw_card, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_radius(sw_card, 20, 0);
  lv_obj_set_style_pad_all(sw_card, 6, 0);
  lv_obj_remove_flag(sw_card, LV_OBJ_FLAG_SCROLLABLE);

  sw_time_label = lv_label_create(sw_card);
  lv_label_set_text(sw_time_label, "00:00.0");
  lv_obj_set_style_text_font(sw_time_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(sw_time_label, lv_color_hex(0xA6E3A1), 0);
  lv_obj_align(sw_time_label, LV_ALIGN_TOP_MID, 0, 0);

  sw_lap_lbl = lv_label_create(sw_card);
  lv_label_set_text(sw_lap_lbl, "Stopwatch Ready");
  lv_obj_set_style_text_font(sw_lap_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(sw_lap_lbl, lv_color_hex(0x89B4FA), 0);
  lv_obj_align(sw_lap_lbl, LV_ALIGN_TOP_MID, 0, 32);

  sw_btn_start = lv_button_create(sw_card);
  lv_obj_set_size(sw_btn_start, 98, 36);
  lv_obj_align(sw_btn_start, LV_ALIGN_BOTTOM_LEFT, 2, -2);
  lv_obj_set_style_bg_color(sw_btn_start, lv_color_hex(0xA6E3A1), 0);
  sw_lbl_start = lv_label_create(sw_btn_start);
  lv_label_set_text(sw_lbl_start, LV_SYMBOL_PLAY " Start");
  lv_obj_set_style_text_color(sw_lbl_start, lv_color_hex(0x11111B), 0);
  lv_obj_center(sw_lbl_start);
  lv_obj_add_event_cb(sw_btn_start, sw_btn_start_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t * sw_btn_lap = lv_button_create(sw_card);
  lv_obj_set_size(sw_btn_lap, 98, 36);
  lv_obj_align(sw_btn_lap, LV_ALIGN_BOTTOM_MID, 0, -2);
  lv_obj_set_style_bg_color(sw_btn_lap, lv_color_hex(0x89B4FA), 0);
  lv_obj_t * sw_lbl_lap = lv_label_create(sw_btn_lap);
  lv_label_set_text(sw_lbl_lap, LV_SYMBOL_LIST " Lap");
  lv_obj_set_style_text_color(sw_lbl_lap, lv_color_hex(0x11111B), 0);
  lv_obj_center(sw_lbl_lap);
  lv_obj_add_event_cb(sw_btn_lap, sw_btn_lap_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t * sw_btn_reset = lv_button_create(sw_card);
  lv_obj_set_size(sw_btn_reset, 98, 36);
  lv_obj_align(sw_btn_reset, LV_ALIGN_BOTTOM_RIGHT, -2, -2);
  lv_obj_set_style_bg_color(sw_btn_reset, lv_color_hex(0xF38BA8), 0);
  lv_obj_t * sw_lbl_reset = lv_label_create(sw_btn_reset);
  lv_label_set_text(sw_lbl_reset, LV_SYMBOL_TRASH " Reset");
  lv_obj_set_style_text_color(sw_lbl_reset, lv_color_hex(0x11111B), 0);
  lv_obj_center(sw_lbl_reset);
  lv_obj_add_event_cb(sw_btn_reset, sw_btn_reset_cb, LV_EVENT_CLICKED, NULL);

  // SECTION 2: COUNTDOWN TIMER
  lv_obj_t * cd_card = lv_obj_create(page_tools);
  lv_obj_set_size(cd_card, SAFE_CARD_W, 116);
  lv_obj_set_style_bg_color(cd_card, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(cd_card, lv_color_hex(0xF9E2AF), 0);
  lv_obj_set_style_radius(cd_card, 20, 0);
  lv_obj_set_style_pad_all(cd_card, 6, 0);
  lv_obj_remove_flag(cd_card, LV_OBJ_FLAG_SCROLLABLE);

  cd_time_label = lv_label_create(cd_card);
  lv_label_set_text(cd_time_label, "01:00");
  lv_obj_set_style_text_font(cd_time_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(cd_time_label, lv_color_hex(0xF9E2AF), 0);
  lv_obj_align(cd_time_label, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t * cd_p1 = lv_button_create(cd_card);
  lv_obj_set_size(cd_p1, 66, 34);
  lv_obj_align(cd_p1, LV_ALIGN_BOTTOM_LEFT, 2, -2);
  lv_obj_set_style_bg_color(cd_p1, lv_color_hex(0x313244), 0);
  lv_obj_t * cd_lp1 = lv_label_create(cd_p1);
  lv_label_set_text(cd_lp1, "+1m");
  lv_obj_center(cd_lp1);
  lv_obj_add_event_cb(cd_p1, cd_btn_preset_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)60000);

  lv_obj_t * cd_p5 = lv_button_create(cd_card);
  lv_obj_set_size(cd_p5, 66, 34);
  lv_obj_align(cd_p5, LV_ALIGN_BOTTOM_LEFT, 72, -2);
  lv_obj_set_style_bg_color(cd_p5, lv_color_hex(0x313244), 0);
  lv_obj_t * cd_lp5 = lv_label_create(cd_p5);
  lv_label_set_text(cd_lp5, "+5m");
  lv_obj_center(cd_lp5);
  lv_obj_add_event_cb(cd_p5, cd_btn_preset_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)300000);

  cd_btn_start = lv_button_create(cd_card);
  lv_obj_set_size(cd_btn_start, 92, 34);
  lv_obj_align(cd_btn_start, LV_ALIGN_BOTTOM_RIGHT, -74, -2);
  lv_obj_set_style_bg_color(cd_btn_start, lv_color_hex(0xF9E2AF), 0);
  cd_lbl_start = lv_label_create(cd_btn_start);
  lv_label_set_text(cd_lbl_start, LV_SYMBOL_PLAY " Start");
  lv_obj_set_style_text_color(cd_lbl_start, lv_color_hex(0x11111B), 0);
  lv_obj_center(cd_lbl_start);
  lv_obj_add_event_cb(cd_btn_start, cd_btn_start_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t * cd_btn_reset = lv_button_create(cd_card);
  lv_obj_set_size(cd_btn_reset, 68, 34);
  lv_obj_align(cd_btn_reset, LV_ALIGN_BOTTOM_RIGHT, -2, -2);
  lv_obj_set_style_bg_color(cd_btn_reset, lv_color_hex(0xF38BA8), 0);
  lv_obj_t * cd_lbl_reset = lv_label_create(cd_btn_reset);
  lv_label_set_text(cd_lbl_reset, LV_SYMBOL_TRASH " Rst");
  lv_obj_set_style_text_color(cd_lbl_reset, lv_color_hex(0x11111B), 0);
  lv_obj_center(cd_lbl_reset);
  lv_obj_add_event_cb(cd_btn_reset, cd_btn_reset_cb, LV_EVENT_CLICKED, NULL);

  // SECTION 3: ALARM CLOCK
  lv_obj_t * alarm_card = lv_obj_create(page_tools);
  lv_obj_set_size(alarm_card, SAFE_CARD_W, 102);
  lv_obj_set_style_bg_color(alarm_card, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(alarm_card, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_radius(alarm_card, 20, 0);
  lv_obj_set_style_pad_all(alarm_card, 6, 0);
  lv_obj_remove_flag(alarm_card, LV_OBJ_FLAG_SCROLLABLE);

  alarm_lbl_time = lv_label_create(alarm_card);
  char abuf[32];
  int h12 = alarm_hour % 12;
  if (h12 == 0) h12 = 12;
  snprintf(abuf, sizeof(abuf), "%02d:%02d %s", h12, alarm_min, alarm_hour >= 12 ? "PM" : "AM");
  lv_label_set_text(alarm_lbl_time, abuf);
  lv_obj_set_style_text_font(alarm_lbl_time, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(alarm_lbl_time, lv_color_hex(0x89B4FA), 0);
  lv_obj_align(alarm_lbl_time, LV_ALIGN_TOP_LEFT, 6, 2);

  alarm_sw_enable = lv_switch_create(alarm_card);
  lv_obj_align(alarm_sw_enable, LV_ALIGN_TOP_RIGHT, -6, 4);
  if (alarm_enabled) lv_obj_add_state(alarm_sw_enable, LV_STATE_CHECKED);
  lv_obj_add_event_cb(alarm_sw_enable, alarm_sw_cb, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t * alarm_btn_set = lv_button_create(alarm_card);
  lv_obj_set_size(alarm_btn_set, 156, 34);
  lv_obj_align(alarm_btn_set, LV_ALIGN_BOTTOM_LEFT, 6, -2);
  lv_obj_set_style_bg_color(alarm_btn_set, lv_color_hex(0x313244), 0);
  lv_obj_t * alarm_lbl_set = lv_label_create(alarm_btn_set);
  lv_label_set_text(alarm_lbl_set, LV_SYMBOL_BELL " +15m Alarm");
  lv_obj_center(alarm_lbl_set);
  lv_obj_add_event_cb(alarm_btn_set, alarm_btn_change_cb, LV_EVENT_CLICKED, NULL);

  // ================= PAGE 2: BLUETOOTH (BLE UART & MESSAGING) =================
  page_bt = lv_obj_create(lv_scr_act());
  lv_obj_set_size(page_bt, SAFE_PAGE_W, SAFE_PAGE_H);
  lv_obj_align(page_bt, LV_ALIGN_TOP_MID, 0, SAFE_PAGE_Y);
  lv_obj_set_style_bg_color(page_bt, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(page_bt, 0, 0);
  lv_obj_set_style_pad_all(page_bt, 0, 0);
  lv_obj_remove_flag(page_bt, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * bt_hdr = lv_obj_create(page_bt);
  lv_obj_set_size(bt_hdr, SAFE_CARD_W, 68);
  lv_obj_align(bt_hdr, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_set_style_bg_color(bt_hdr, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(bt_hdr, lv_color_hex(0xCBA6F7), 0);
  lv_obj_set_style_border_width(bt_hdr, 2, 0);
  lv_obj_set_style_radius(bt_hdr, 18, 0);
  lv_obj_set_style_pad_all(bt_hdr, 6, 0);
  lv_obj_remove_flag(bt_hdr, LV_OBJ_FLAG_SCROLLABLE);

  bt_status_lbl = lv_label_create(bt_hdr);
  lv_label_set_text(bt_status_lbl, "Advertising: " BLE_DEVICE_NAME);
  lv_obj_set_style_text_font(bt_status_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bt_status_lbl, lv_color_hex(0x89B4FA), 0);
  lv_obj_align(bt_status_lbl, LV_ALIGN_TOP_LEFT, 4, 2);

  bt_sub_lbl = lv_label_create(bt_hdr);
  lv_label_set_text(bt_sub_lbl, "Android BLE Requests • Chat & Sync");
  lv_obj_set_style_text_font(bt_sub_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bt_sub_lbl, lv_color_hex(0xA6ADC8), 0);
  lv_obj_align(bt_sub_lbl, LV_ALIGN_BOTTOM_LEFT, 4, -2);

  bt_switch = lv_switch_create(bt_hdr);
  lv_obj_align(bt_switch, LV_ALIGN_RIGHT_MID, -4, 0);
  if (ble_power_on) lv_obj_add_state(bt_switch, LV_STATE_CHECKED);
  lv_obj_add_event_cb(bt_switch, bt_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // Action Buttons Row: Compose Message, Sync with Android, Clear Log
  lv_obj_t * btn_bt_msg = lv_button_create(page_bt);
  lv_obj_set_size(btn_bt_msg, 104, 36);
  lv_obj_align(btn_bt_msg, LV_ALIGN_TOP_LEFT, 8, 76);
  lv_obj_set_style_bg_color(btn_bt_msg, lv_color_hex(0xCBA6F7), 0);
  lv_obj_set_style_radius(btn_bt_msg, 16, 0);
  lv_obj_t * lbl_bt_msg = lv_label_create(btn_bt_msg);
  lv_label_set_text(lbl_bt_msg, LV_SYMBOL_EDIT " Msg");
  lv_obj_set_style_text_color(lbl_bt_msg, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_bt_msg);
  lv_obj_add_event_cb(btn_bt_msg, [](lv_event_t * e) {
    open_bt_compose_modal();
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_bt_ping = lv_button_create(page_bt);
  lv_obj_set_size(btn_bt_ping, 118, 36);
  lv_obj_align(btn_bt_ping, LV_ALIGN_TOP_MID, 6, 76);
  lv_obj_set_style_bg_color(btn_bt_ping, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_radius(btn_bt_ping, 16, 0);
  lv_obj_t * lbl_bt_ping = lv_label_create(btn_bt_ping);
  lv_label_set_text(lbl_bt_ping, LV_SYMBOL_REFRESH " Sync BT");
  lv_obj_set_style_text_color(lbl_bt_ping, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_bt_ping);
  lv_obj_add_event_cb(btn_bt_ping, [](lv_event_t * e) {
    play_beep(950, 50);
    ble_manual_sync_pending = true;
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * btn_bt_clr = lv_button_create(page_bt);
  lv_obj_set_size(btn_bt_clr, 92, 36);
  lv_obj_align(btn_bt_clr, LV_ALIGN_TOP_RIGHT, -8, 76);
  lv_obj_set_style_bg_color(btn_bt_clr, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_radius(btn_bt_clr, 16, 0);
  lv_obj_t * lbl_bt_clr = lv_label_create(btn_bt_clr);
  lv_label_set_text(lbl_bt_clr, LV_SYMBOL_TRASH " Clr");
  lv_obj_set_style_text_color(lbl_bt_clr, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_bt_clr);
  lv_obj_add_event_cb(btn_bt_clr, [](lv_event_t * e) {
    play_beep(450, 50);
    bt_msg_count = 0;
    bt_log_dirty = false;
    refresh_bt_message_list();
  }, LV_EVENT_CLICKED, NULL);

  bt_msg_list = lv_obj_create(page_bt);
  lv_obj_set_size(bt_msg_list, SAFE_CARD_W, 276);
  lv_obj_align(bt_msg_list, LV_ALIGN_TOP_MID, 0, 118);
  lv_obj_set_style_bg_color(bt_msg_list, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(bt_msg_list, lv_color_hex(0x313244), 0);
  lv_obj_set_style_border_width(bt_msg_list, 2, 0);
  lv_obj_set_style_radius(bt_msg_list, 18, 0);
  lv_obj_set_style_pad_all(bt_msg_list, 8, 0);
  lv_obj_set_scroll_dir(bt_msg_list, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(bt_msg_list, LV_SCROLLBAR_MODE_AUTO);

  bt_log_label = lv_label_create(bt_msg_list);
  lv_obj_set_width(bt_log_label, SAFE_CARD_W - 22);
  lv_label_set_long_mode(bt_log_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_font(bt_log_label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bt_log_label, lv_color_hex(0xCDD6F4), 0);
  bt_log_dirty = false;
  refresh_bt_message_list();

  // ================= PAGE 3: DEVICE SETTINGS & BOUNDARY MONITOR =================
  page_settings = lv_obj_create(lv_scr_act());
  lv_obj_set_size(page_settings, SAFE_PAGE_W, SAFE_PAGE_H);
  lv_obj_align(page_settings, LV_ALIGN_TOP_MID, 0, SAFE_PAGE_Y);
  lv_obj_set_style_bg_color(page_settings, lv_color_hex(0x11111B), 0);
  lv_obj_set_style_border_width(page_settings, 0, 0);
  lv_obj_set_style_pad_all(page_settings, 4, 0);
  lv_obj_set_style_pad_row(page_settings, 6, 0);
  lv_obj_set_flex_flow(page_settings, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(page_settings, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  // Make Settings vertically scrollable so all cards + Boundary Monitor fit cleanly inside the bezel
  lv_obj_set_scroll_dir(page_settings, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(page_settings, LV_SCROLLBAR_MODE_AUTO);

  // Live Computational & Memory Boundaries Card in Settings
  lv_obj_t * bound_card = lv_obj_create(page_settings);
  lv_obj_set_size(bound_card, SAFE_CARD_W, 138);
  lv_obj_set_style_bg_color(bound_card, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(bound_card, lv_color_hex(0x00E5FF), 0);
  lv_obj_set_style_border_width(bound_card, 2, 0);
  lv_obj_set_style_radius(bound_card, 18, 0);
  lv_obj_set_style_pad_all(bound_card, 6, 0);
  lv_obj_remove_flag(bound_card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * bound_hdr = lv_label_create(bound_card);
  lv_label_set_text(bound_hdr, "HW BOUNDARIES");
  lv_obj_set_style_text_font(bound_hdr, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bound_hdr, lv_color_hex(0x00E5FF), 0);
  lv_obj_align(bound_hdr, LV_ALIGN_TOP_LEFT, 4, 0);

  boundary_status_badge = lv_label_create(bound_card);
  lv_label_set_text(boundary_status_badge, "OK (Safe Headroom)");
  lv_obj_set_style_text_font(boundary_status_badge, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(boundary_status_badge, lv_color_hex(0xA6E3A1), 0);
  lv_obj_align(boundary_status_badge, LV_ALIGN_TOP_RIGHT, -4, 0);

  boundary_info_lbl = lv_label_create(bound_card);
  char init_rep[192];
  format_boundary_report(init_rep, sizeof(init_rep));
  lv_label_set_text(boundary_info_lbl, init_rep);
  lv_obj_set_style_text_font(boundary_info_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(boundary_info_lbl, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(boundary_info_lbl, LV_ALIGN_TOP_LEFT, 4, 22);

  lv_obj_t * btn_bound_test = lv_button_create(bound_card);
  lv_obj_set_size(btn_bound_test, 314, 32);
  lv_obj_align(btn_bound_test, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(btn_bound_test, lv_color_hex(0x00E5FF), 0);
  lv_obj_set_style_radius(btn_bound_test, 12, 0);
  lv_obj_t * lbl_bound_test = lv_label_create(btn_bound_test);
  lv_label_set_text(lbl_bound_test, LV_SYMBOL_REFRESH " Run Boundary & Heap Self-Test");
  lv_obj_set_style_text_color(lbl_bound_test, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_bound_test);
  lv_obj_add_event_cb(btn_bound_test, [](lv_event_t * e) {
    run_boundary_self_test();
  }, LV_EVENT_CLICKED, NULL);

  // Manual Clock & Date Card in Settings
  lv_obj_t * time_cfg_card = lv_obj_create(page_settings);
  lv_obj_set_size(time_cfg_card, SAFE_CARD_W, 92);
  lv_obj_set_style_bg_color(time_cfg_card, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(time_cfg_card, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_border_width(time_cfg_card, 2, 0);
  lv_obj_set_style_radius(time_cfg_card, 18, 0);
  lv_obj_set_style_pad_all(time_cfg_card, 6, 0);
  lv_obj_remove_flag(time_cfg_card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * btn_open_tm = lv_button_create(time_cfg_card);
  lv_obj_set_size(btn_open_tm, 314, 36);
  lv_obj_align(btn_open_tm, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(btn_open_tm, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_radius(btn_open_tm, 14, 0);
  lv_obj_t * lbl_open_tm = lv_label_create(btn_open_tm);
  lv_label_set_text(lbl_open_tm, LV_SYMBOL_EDIT "  Set Time & Date Manually");
  lv_obj_set_style_text_color(lbl_open_tm, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_open_tm);
  lv_obj_add_event_cb(btn_open_tm, [](lv_event_t * e) {
    open_manual_time_modal();
  }, LV_EVENT_CLICKED, NULL);

  lv_obj_t * fmt_lbl = lv_label_create(time_cfg_card);
  lv_label_set_text(fmt_lbl, "12-Hour (AM/PM) Format:");
  lv_obj_set_style_text_font(fmt_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(fmt_lbl, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(fmt_lbl, LV_ALIGN_BOTTOM_LEFT, 4, -2);

  format_12h_switch = lv_switch_create(time_cfg_card);
  lv_obj_align(format_12h_switch, LV_ALIGN_BOTTOM_RIGHT, -4, 0);
  if (use_12h_format) lv_obj_add_state(format_12h_switch, LV_STATE_CHECKED);
  lv_obj_add_event_cb(format_12h_switch, format_12h_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // Watch Face Selection Card in Settings
  lv_obj_t * face_card = lv_obj_create(page_settings);
  lv_obj_set_size(face_card, SAFE_CARD_W, 82);
  lv_obj_set_style_bg_color(face_card, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(face_card, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_border_width(face_card, 2, 0);
  lv_obj_set_style_radius(face_card, 18, 0);
  lv_obj_set_style_pad_all(face_card, 6, 0);
  lv_obj_remove_flag(face_card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * face_card_lbl = lv_label_create(face_card);
  lv_label_set_text(face_card_lbl, "Default Watch Face:");
  lv_obj_set_style_text_font(face_card_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(face_card_lbl, lv_color_hex(0x89B4FA), 0);
  lv_obj_align(face_card_lbl, LV_ALIGN_TOP_LEFT, 4, 0);

  settings_face_dropdown = lv_dropdown_create(face_card);
  lv_dropdown_clear_options(settings_face_dropdown);
  for (int i = 0; i < num_faces; i++) {
    lv_dropdown_add_option(settings_face_dropdown, face_names[i], i);
  }
  lv_dropdown_set_selected(settings_face_dropdown, selected_face_idx);
  lv_obj_set_size(settings_face_dropdown, 314, 34);
  lv_obj_align(settings_face_dropdown, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(settings_face_dropdown, lv_color_hex(0x313244), 0);
  lv_obj_set_style_text_color(settings_face_dropdown, lv_color_hex(0xCDD6F4), 0);
  lv_obj_set_style_radius(settings_face_dropdown, 14, 0);
  lv_obj_add_event_cb(settings_face_dropdown, face_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // Brightness Control Card
  lv_obj_t * bright_card = lv_obj_create(page_settings);
  lv_obj_set_size(bright_card, SAFE_CARD_W, 84);
  lv_obj_set_style_bg_color(bright_card, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(bright_card, lv_color_hex(0xF9E2AF), 0);
  lv_obj_set_style_border_width(bright_card, 2, 0);
  lv_obj_set_style_radius(bright_card, 18, 0);
  lv_obj_set_style_pad_all(bright_card, 6, 0);
  lv_obj_remove_flag(bright_card, LV_OBJ_FLAG_SCROLLABLE);

  brightness_lbl_val = lv_label_create(bright_card);
  char btxt[32];
  int bright_pct = (display_brightness * 100) / 255;
  snprintf(btxt, sizeof(btxt), "Brightness: %d%%", bright_pct);
  lv_label_set_text(brightness_lbl_val, btxt);
  lv_obj_set_style_text_font(brightness_lbl_val, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(brightness_lbl_val, lv_color_hex(0xF9E2AF), 0);
  lv_obj_align(brightness_lbl_val, LV_ALIGN_TOP_MID, 0, 2);

  brightness_slider = lv_slider_create(bright_card);
  lv_slider_set_range(brightness_slider, 20, 255);
  lv_slider_set_value(brightness_slider, display_brightness, LV_ANIM_OFF);
  lv_obj_set_size(brightness_slider, 286, 16);
  lv_obj_align(brightness_slider, LV_ALIGN_BOTTOM_MID, 0, -6);
  lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0x313244), 0);
  lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0xF9E2AF), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0xCDD6F4), LV_PART_KNOB);
  lv_obj_add_event_cb(brightness_slider, brightness_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // Audio Control Card
  lv_obj_t * audio_card = lv_obj_create(page_settings);
  lv_obj_set_size(audio_card, SAFE_CARD_W, 58);
  lv_obj_set_style_bg_color(audio_card, lv_color_hex(0x1E1E2E), 0);
  lv_obj_set_style_border_color(audio_card, lv_color_hex(0x89B4FA), 0);
  lv_obj_set_style_border_width(audio_card, 2, 0);
  lv_obj_set_style_radius(audio_card, 18, 0);
  lv_obj_set_style_pad_all(audio_card, 6, 0);
  lv_obj_remove_flag(audio_card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * audio_lbl = lv_label_create(audio_card);
  lv_label_set_text(audio_lbl, "System Audio Beeps:");
  lv_obj_set_style_text_font(audio_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(audio_lbl, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(audio_lbl, LV_ALIGN_LEFT_MID, 4, 0);

  audio_switch = lv_switch_create(audio_card);
  lv_obj_align(audio_switch, LV_ALIGN_RIGHT_MID, -4, 0);
  if (!audio_muted) lv_obj_add_state(audio_switch, LV_STATE_CHECKED);
  lv_obj_add_event_cb(audio_switch, audio_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // ---------------- WATCH FACE 4: ASCII MATRIX TERMINAL ----------------
  face_container_ascii = lv_obj_create(page_clock);
  lv_obj_set_size(face_container_ascii, SAFE_FACE_W, SAFE_FACE_H);
  lv_obj_align(face_container_ascii, LV_ALIGN_TOP_MID, 0, 46);
  lv_obj_set_style_bg_color(face_container_ascii, lv_color_hex(0x050D05), 0);
  lv_obj_set_style_border_color(face_container_ascii, lv_color_hex(0x00FF66), 0);
  lv_obj_set_style_border_width(face_container_ascii, 2, 0);
  lv_obj_set_style_radius(face_container_ascii, 24, 0);
  lv_obj_set_flex_flow(face_container_ascii, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(face_container_ascii, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(face_container_ascii, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * ascii_hdr = lv_label_create(face_container_ascii);
  lv_label_set_text(ascii_hdr, "> SYSTEM_TERMINAL v2.6");
  lv_obj_set_style_text_font(ascii_hdr, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(ascii_hdr, lv_color_hex(0x00FF66), 0);

  lv_obj_t * ascii_card = lv_obj_create(face_container_ascii);
  lv_obj_set_size(ascii_card, 320, 196);
  lv_obj_set_style_bg_color(ascii_card, lv_color_hex(0x0A1A0A), 0);
  lv_obj_set_style_border_color(ascii_card, lv_color_hex(0x00FF66), 0);
  lv_obj_set_style_border_width(ascii_card, 1, 0);
  lv_obj_set_style_radius(ascii_card, 16, 0);
  lv_obj_set_flex_flow(ascii_card, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(ascii_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(ascii_card, LV_OBJ_FLAG_SCROLLABLE);

  for (int i = 0; i < 5; i++) {
    ascii_lines[i] = lv_label_create(ascii_card);
    lv_label_set_text(ascii_lines[i], "███  ███  :  ███  ███");
    lv_obj_set_style_text_font(ascii_lines[i], &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ascii_lines[i], lv_color_hex(0x00FF66), 0);
  }

  ascii_sub_info = lv_label_create(face_container_ascii);
  lv_label_set_text(ascii_sub_info, "SEC:00  |  2026.08.05  |  AM");
  lv_obj_set_style_text_font(ascii_sub_info, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(ascii_sub_info, lv_color_hex(0x00F0FF), 0);

  // ---------------- WATCH FACE 5: WEATHER STATION ----------------
  face_container_weather = lv_obj_create(page_clock);
  lv_obj_set_size(face_container_weather, SAFE_FACE_W, SAFE_FACE_H);
  lv_obj_align(face_container_weather, LV_ALIGN_TOP_MID, 0, 46);
  lv_obj_set_style_bg_color(face_container_weather, lv_color_hex(0x0F172A), 0);
  lv_obj_set_style_border_color(face_container_weather, lv_color_hex(0x38BDF8), 0);
  lv_obj_set_style_border_width(face_container_weather, 2, 0);
  lv_obj_set_style_radius(face_container_weather, 24, 0);
  lv_obj_set_style_pad_all(face_container_weather, 6, 0);
  lv_obj_set_style_pad_row(face_container_weather, 6, 0);
  lv_obj_set_flex_flow(face_container_weather, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(face_container_weather, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(face_container_weather, LV_OBJ_FLAG_SCROLLABLE);

  // Time & Location Card
  lv_obj_t * w_time_card = lv_obj_create(face_container_weather);
  lv_obj_set_size(w_time_card, 324, 66);
  lv_obj_set_style_bg_color(w_time_card, lv_color_hex(0x1E293B), 0);
  lv_obj_set_style_border_color(w_time_card, lv_color_hex(0x38BDF8), 0);
  lv_obj_set_style_border_width(w_time_card, 1, 0);
  lv_obj_set_style_radius(w_time_card, 14, 0);
  lv_obj_remove_flag(w_time_card, LV_OBJ_FLAG_SCROLLABLE);

  weather_time_lbl = lv_label_create(w_time_card);
  lv_label_set_text(weather_time_lbl, "--:--");
  lv_obj_set_style_text_font(weather_time_lbl, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(weather_time_lbl, lv_color_hex(0x38BDF8), 0);
  lv_obj_align(weather_time_lbl, LV_ALIGN_TOP_MID, 0, -2);

  weather_date_lbl = lv_label_create(w_time_card);
  lv_label_set_text(weather_date_lbl, "Initializing Weather...");
  lv_obj_set_style_text_font(weather_date_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(weather_date_lbl, lv_color_hex(0x94A3B8), 0);
  lv_obj_align(weather_date_lbl, LV_ALIGN_BOTTOM_MID, 0, 2);

  // Temperature & Main Weather Card
  lv_obj_t * w_main_card = lv_obj_create(face_container_weather);
  lv_obj_set_size(w_main_card, 324, 104);
  lv_obj_set_style_bg_color(w_main_card, lv_color_hex(0x1E293B), 0);
  lv_obj_set_style_border_color(w_main_card, lv_color_hex(0xF59E0B), 0);
  lv_obj_set_style_border_width(w_main_card, 1, 0);
  lv_obj_set_style_radius(w_main_card, 14, 0);
  lv_obj_remove_flag(w_main_card, LV_OBJ_FLAG_SCROLLABLE);

  weather_temp_c_lbl = lv_label_create(w_main_card);
  lv_label_set_text(weather_temp_c_lbl, "--°C");
  lv_obj_set_style_text_font(weather_temp_c_lbl, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(weather_temp_c_lbl, lv_color_hex(0xF59E0B), 0);
  lv_obj_align(weather_temp_c_lbl, LV_ALIGN_TOP_LEFT, 8, 2);

  weather_temp_f_lbl = lv_label_create(w_main_card);
  lv_label_set_text(weather_temp_f_lbl, "(--°F)");
  lv_obj_set_style_text_font(weather_temp_f_lbl, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(weather_temp_f_lbl, lv_color_hex(0xFBBF24), 0);
  lv_obj_align(weather_temp_f_lbl, LV_ALIGN_TOP_LEFT, 144, 14);

  weather_cond_lbl = lv_label_create(w_main_card);
  lv_label_set_text(weather_cond_lbl, "No Weather Data");
  lv_obj_set_style_text_font(weather_cond_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(weather_cond_lbl, lv_color_hex(0xE2E8F0), 0);
  lv_obj_align(weather_cond_lbl, LV_ALIGN_BOTTOM_LEFT, 8, -6);

  // Weather Details Card
  lv_obj_t * w_det_card = lv_obj_create(face_container_weather);
  lv_obj_set_size(w_det_card, 324, 68);
  lv_obj_set_style_bg_color(w_det_card, lv_color_hex(0x1E293B), 0);
  lv_obj_set_style_border_color(w_det_card, lv_color_hex(0x10B981), 0);
  lv_obj_set_style_border_width(w_det_card, 1, 0);
  lv_obj_set_style_radius(w_det_card, 14, 0);
  lv_obj_remove_flag(w_det_card, LV_OBJ_FLAG_SCROLLABLE);

  weather_details_lbl = lv_label_create(w_det_card);
  lv_label_set_text(weather_details_lbl, "Tap 'Refresh' below");
  lv_obj_set_style_text_font(weather_details_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(weather_details_lbl, lv_color_hex(0x6EE7B7), 0);
  lv_obj_align(weather_details_lbl, LV_ALIGN_CENTER, 0, 0);

  // Refresh Button & Status Row
  lv_obj_t * w_action_row = lv_obj_create(face_container_weather);
  lv_obj_set_size(w_action_row, 324, 44);
  lv_obj_set_style_bg_color(w_action_row, lv_color_hex(0x0F172A), 0);
  lv_obj_set_style_border_width(w_action_row, 0, 0);
  lv_obj_set_style_pad_all(w_action_row, 2, 0);
  lv_obj_remove_flag(w_action_row, LV_OBJ_FLAG_SCROLLABLE);

  weather_btn_refresh = lv_button_create(w_action_row);
  lv_obj_set_size(weather_btn_refresh, 144, 34);
  lv_obj_align(weather_btn_refresh, LV_ALIGN_LEFT_MID, 0, 0);
  lv_obj_set_style_bg_color(weather_btn_refresh, lv_color_hex(0x38BDF8), 0);
  lv_obj_set_style_radius(weather_btn_refresh, 12, 0);

  lv_obj_t * w_lbl_ref = lv_label_create(weather_btn_refresh);
  lv_label_set_text(w_lbl_ref, LV_SYMBOL_REFRESH " Refresh");
  lv_obj_set_style_text_color(w_lbl_ref, lv_color_hex(0x0F172A), 0);
  lv_obj_center(w_lbl_ref);
  lv_obj_add_event_cb(weather_btn_refresh, weather_refresh_btn_cb, LV_EVENT_CLICKED, NULL);

  weather_status_lbl = lv_label_create(w_action_row);
  lv_label_set_text(weather_status_lbl, "Tap to sync");
  lv_obj_set_style_text_font(weather_status_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(weather_status_lbl, lv_color_hex(0x94A3B8), 0);
  lv_obj_align(weather_status_lbl, LV_ALIGN_RIGHT_MID, 0, 0);

  // ---------------- WATCH FACE 6: AMOLED SPORT ARC ----------------
  face_container_amoled_arc = lv_obj_create(page_clock);
  lv_obj_set_size(face_container_amoled_arc, SAFE_FACE_W, SAFE_FACE_H);
  lv_obj_align(face_container_amoled_arc, LV_ALIGN_TOP_MID, 0, 46);
  lv_obj_set_style_bg_color(face_container_amoled_arc, lv_color_hex(0x000000), 0);
  lv_obj_set_style_border_width(face_container_amoled_arc, 0, 0);
  lv_obj_set_style_radius(face_container_amoled_arc, 24, 0);
  lv_obj_remove_flag(face_container_amoled_arc, LV_OBJ_FLAG_SCROLLABLE);

  arc_battery_arc = lv_arc_create(face_container_amoled_arc);
  lv_obj_set_size(arc_battery_arc, 316, 316);
  lv_obj_align(arc_battery_arc, LV_ALIGN_CENTER, 0, -10);
  lv_arc_set_bg_angles(arc_battery_arc, 135, 45);
  lv_arc_set_range(arc_battery_arc, 0, 100);
  lv_arc_set_value(arc_battery_arc, 85);
  lv_obj_remove_style(arc_battery_arc, NULL, LV_PART_KNOB);
  lv_obj_remove_flag(arc_battery_arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(arc_battery_arc, lv_color_hex(0x1A1A1A), LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc_battery_arc, 8, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc_battery_arc, lv_color_hex(0x00E5FF), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc_battery_arc, 8, LV_PART_INDICATOR);

  arc_time_lbl = lv_label_create(face_container_amoled_arc);
  lv_label_set_text(arc_time_lbl, "10:42");
  lv_obj_set_style_text_font(arc_time_lbl, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(arc_time_lbl, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(arc_time_lbl, LV_ALIGN_CENTER, 0, -45);

  arc_date_lbl = lv_label_create(face_container_amoled_arc);
  lv_label_set_text(arc_date_lbl, "SUN, OCT 18");
  lv_obj_set_style_text_font(arc_date_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(arc_date_lbl, lv_color_hex(0x888888), 0);
  lv_obj_align(arc_date_lbl, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t *arc_stats_cont = lv_obj_create(face_container_amoled_arc);
  lv_obj_set_size(arc_stats_cont, 316, 84);
  lv_obj_align(arc_stats_cont, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_bg_color(arc_stats_cont, lv_color_hex(0x0B0B0B), 0);
  lv_obj_set_style_border_color(arc_stats_cont, lv_color_hex(0x222222), 0);
  lv_obj_set_style_border_width(arc_stats_cont, 1, 0);
  lv_obj_set_style_radius(arc_stats_cont, 18, 0);
  lv_obj_set_flex_flow(arc_stats_cont, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(arc_stats_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(arc_stats_cont, LV_OBJ_FLAG_SCROLLABLE);

  arc_steps_lbl = lv_label_create(arc_stats_cont);
  lv_label_set_text(arc_steps_lbl, "STEPS\n" LV_SYMBOL_RIGHT " 8,432");
  lv_obj_set_style_text_color(arc_steps_lbl, lv_color_hex(0x00FF88), 0);

  arc_hr_lbl = lv_label_create(arc_stats_cont);
  lv_label_set_text(arc_hr_lbl, "PULSE\n" LV_SYMBOL_CHARGE " 72 BPM");
  lv_obj_set_style_text_color(arc_hr_lbl, lv_color_hex(0xFF0055), 0);

  // Register Touch Swipe Gestures across all 4 Pages & Long-Press Watch Face Switcher
  lv_obj_t * pages[] = {page_clock, page_tools, page_bt, page_settings};
  for (int i = 0; i < NUM_PAGES; i++) {
    if (pages[i]) {
      lv_obj_add_flag(pages[i], LV_OBJ_FLAG_CLICKABLE);
      lv_obj_add_event_cb(pages[i], screen_gesture_event_cb, LV_EVENT_GESTURE, NULL);
    }
  }

  lv_obj_t * face_containers[] = {
    face_container_modern,
    face_container_sundial,
    face_container_neon,
    face_container_analog,
    face_container_ascii,
    face_container_weather,
    face_container_amoled_arc
  };
  for (int i = 0; i < num_faces; i++) {
    if (face_containers[i]) {
      lv_obj_remove_flag(face_containers[i], LV_OBJ_FLAG_SCROLLABLE);
      lv_obj_add_flag(face_containers[i], LV_OBJ_FLAG_CLICKABLE);
      lv_obj_add_event_cb(face_containers[i], watch_face_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);
      lv_obj_add_event_cb(face_containers[i], screen_gesture_event_cb, LV_EVENT_GESTURE, NULL);
    }
  }

  // Ensure quick_bar and top_bar are always in the foreground so nothing covers buttons
  lv_obj_move_foreground(quick_bar);
  lv_obj_move_foreground(top_bar);

  // Show default page and active face
  switch_to_page(0);
}

// --- FACE 6: AMOLED SPORT ARC DISPLAY UPDATE ---
void update_amoled_arc_display(int hour, int minute, int sec, int year, int month, int day) {
  (void)sec;
  if (!face_container_amoled_arc) return;

  int disp_h = hour;
  const char* ampm = "";
  if (use_12h_format) {
    disp_h = hour % 12;
    if (disp_h == 0) disp_h = 12;
    ampm = (hour >= 12) ? " PM" : " AM";
  }

  if (arc_time_lbl) {
    char tbuf[24];
    snprintf(tbuf, sizeof(tbuf), "%02d:%02d%s", disp_h, minute, ampm);
    lv_label_set_text(arc_time_lbl, tbuf);
  }

  if (arc_date_lbl) {
    static const char* mnames[] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
    const char* mstr = (month >= 1 && month <= 12) ? mnames[month - 1] : "OCT";
    char dbuf[32];
    snprintf(dbuf, sizeof(dbuf), "%s %d, %04d", mstr, day, year);
    lv_label_set_text(arc_date_lbl, dbuf);
  }

  if (arc_battery_arc) {
    int pct = (pmu_available && power.isBatteryConnect()) ? power.getBatteryPercent() : 100;
    lv_arc_set_value(arc_battery_arc, pct);
  }

  if (arc_steps_lbl) {
    char sbuf[40];
    snprintf(sbuf, sizeof(sbuf), "STEPS\n" LV_SYMBOL_RIGHT " %lu", (unsigned long)watch_steps_count);
    lv_label_set_text(arc_steps_lbl, sbuf);
  }

  if (arc_hr_lbl) {
    char hbuf[40];
    snprintf(hbuf, sizeof(hbuf), "PULSE\n" LV_SYMBOL_CHARGE " %d BPM", watch_heart_rate_bpm);
    lv_label_set_text(arc_hr_lbl, hbuf);
  }
}

// --- Periodic Display & Tools Updates ---
void update_clock_display() {
  if (millis() - last_clock_update < 500) return;
  last_clock_update = millis();

  int year = 2026, month = 10, day = 18, hour = 12, min_val = 0, sec = 0;
  get_watch_datetime(year, month, day, hour, min_val, sec);

  char time_str[32] = "--:--:--";
  char bar_time_str[32] = "--:--";
  char ampm_str[8] = "";
  char date_str[64] = "Initializing...";

  if (use_12h_format) {
    int h12 = hour % 12;
    if (h12 == 0) h12 = 12;
    const char* ampm = (hour >= 12) ? "PM" : "AM";
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", h12, min_val, sec);
    snprintf(bar_time_str, sizeof(bar_time_str), "%d:%02d %s", h12, min_val, ampm);
    snprintf(ampm_str, sizeof(ampm_str), "%s", ampm);
  } else {
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hour, min_val, sec);
    snprintf(bar_time_str, sizeof(bar_time_str), "%02d:%02d", hour, min_val);
    snprintf(ampm_str, sizeof(ampm_str), "");
  }

  snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d  [%s]",
           year, month, day, tz_options[current_tz_idx].name);

  if (alarm_enabled) {
    if (hour == alarm_hour && min_val == alarm_min && sec == 0 && !alarm_triggered_today) {
      alarm_triggered_today = true;
      trigger_alert_popup("ALARM!", "Time to wake up!");
    }
    if (hour != alarm_hour || min_val != alarm_min) {
      alarm_triggered_today = false;
    }
  }

  if (clock_time_label) lv_label_set_text(clock_time_label, time_str);
  if (clock_ampm_label) lv_label_set_text(clock_ampm_label, ampm_str);
  if (clock_date_label) lv_label_set_text(clock_date_label, date_str);
  if (bar_time_lbl) lv_label_set_text(bar_time_lbl, bar_time_str);

  // Update Active Face Graphics
  if (active_page_idx == 0) {
    if (selected_face_idx == 1) {
      update_sundial_display();
    } else if (selected_face_idx == 2) {
      update_neon_display(hour, min_val, sec, year, month, day);
    } else if (selected_face_idx == 3) {
      update_analog_display(hour, min_val, sec);
    } else if (selected_face_idx == 4) {
      update_ascii_display(hour, min_val, sec, year, month, day);
    } else if (selected_face_idx == 5) {
      update_weather_display();
    } else if (selected_face_idx == 6) {
      update_amoled_arc_display(hour, min_val, sec, year, month, day);
    }
  }

  if (wifi_status_label) {
    lv_label_set_text(wifi_status_label, last_android_req_str);
  }
}

void update_tools_display() {
  if (sw_running && sw_time_label) {
    uint32_t total_ms = sw_accumulated_ms + (millis() - sw_start_time);
    int min_val = (total_ms / 60000) % 60;
    int sec = (total_ms / 1000) % 60;
    int tenths = (total_ms / 100) % 10;
    char sbuf[32];
    snprintf(sbuf, sizeof(sbuf), "%02d:%02d.%d", min_val, sec, tenths);
    lv_label_set_text(sw_time_label, sbuf);
  }

  if (cd_running) {
    uint32_t elapsed = millis() - cd_start_time;
    if (elapsed >= cd_duration_ms) {
      cd_running = false;
      cd_remaining_ms = 0;
      if (cd_time_label) lv_label_set_text(cd_time_label, "00:00");
      if (cd_lbl_start) lv_label_set_text(cd_lbl_start, LV_SYMBOL_PLAY " Start");
      trigger_alert_popup("TIMER DONE!", "Countdown Finished!");
    } else {
      cd_remaining_ms = cd_duration_ms - elapsed;
      if (cd_time_label) {
        int min_val = (cd_remaining_ms / 60000) % 60;
        int sec = (cd_remaining_ms / 1000) % 60;
        char cbuf[32];
        snprintf(cbuf, sizeof(cbuf), "%02d:%02d", min_val, sec);
        lv_label_set_text(cd_time_label, cbuf);
      }
    }
  } else if (cd_time_label) {
    int min_val = (cd_remaining_ms / 60000) % 60;
    int sec = (cd_remaining_ms / 1000) % 60;
    char cbuf[32];
    snprintf(cbuf, sizeof(cbuf), "%02d:%02d", min_val, sec);
    lv_label_set_text(cd_time_label, cbuf);
  }
}

void update_pmu_display() {
  if (millis() - last_pmu_update < 3000) return;
  last_pmu_update = millis();

  if (pmu_available) {
    uint16_t vbatt = power.getBattVoltage();
    bool is_charging = power.isCharging();
    int pct = power.isBatteryConnect() ? power.getBatteryPercent() : 100;

    if (batt_status_label) {
      char pbuf[96];
      snprintf(pbuf, sizeof(pbuf), "Battery: %d%% (%dmV) %s",
               pct, vbatt, is_charging ? "Charging" : "");
      lv_label_set_text(batt_status_label, pbuf);
    }

    if (bar_batt_lbl) {
      char bbuf[32];
      snprintf(bbuf, sizeof(bbuf), "%s%d%%", is_charging ? LV_SYMBOL_CHARGE : "", pct);
      lv_label_set_text(bar_batt_lbl, bbuf);
    }
  }
}

// --- Startup Ready Banner ---
static lv_obj_t *ready_banner = NULL;

static void close_ready_banner_timer_cb(lv_timer_t *t) {
  if (ready_banner) {
    lv_obj_delete_async(ready_banner);
    ready_banner = NULL;
  }
  lv_timer_delete(t);
}

void show_startup_ready_banner(bool touch_ok) {
  if (ready_banner) return;
  ready_banner = lv_obj_create(lv_scr_act());
  lv_obj_set_size(ready_banner, 336, 200);
  lv_obj_center(ready_banner);
  lv_obj_set_style_bg_color(ready_banner, lv_color_hex(0x181825), 0);
  lv_obj_set_style_border_color(ready_banner, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_border_width(ready_banner, 3, 0);
  lv_obj_set_style_radius(ready_banner, 22, 0);
  lv_obj_set_style_pad_all(ready_banner, 10, 0);
  lv_obj_remove_flag(ready_banner, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *t_lbl = lv_label_create(ready_banner);
  lv_label_set_text(t_lbl, LV_SYMBOL_OK " WATCH READY");
  lv_obj_set_style_text_font(t_lbl, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(t_lbl, lv_color_hex(0xA6E3A1), 0);
  lv_obj_align(t_lbl, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t *m_lbl = lv_label_create(ready_banner);
  char status_txt[160];
  snprintf(status_txt, sizeof(status_txt),
           "Touch Controller: %s\n"
           "Bluetooth: " BLE_DEVICE_NAME "\n"
           "Safe-Area + Boundary Guard Active",
           touch_ok ? "ONLINE (FT3168)" : "POLLING MODE");
  lv_label_set_text(m_lbl, status_txt);
  lv_obj_set_style_text_align(m_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(m_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(m_lbl, lv_color_hex(0xCDD6F4), 0);
  lv_obj_align(m_lbl, LV_ALIGN_CENTER, 0, -6);

  lv_obj_t *btn_ok = lv_button_create(ready_banner);
  lv_obj_set_size(btn_ok, 230, 40);
  lv_obj_align(btn_ok, LV_ALIGN_BOTTOM_MID, 0, -2);
  lv_obj_set_style_bg_color(btn_ok, lv_color_hex(0xA6E3A1), 0);
  lv_obj_set_style_radius(btn_ok, 16, 0);
  lv_obj_t *lbl_ok = lv_label_create(btn_ok);
  lv_label_set_text(lbl_ok, LV_SYMBOL_OK " Tap to Test / Start");
  lv_obj_set_style_text_color(lbl_ok, lv_color_hex(0x11111B), 0);
  lv_obj_center(lbl_ok);

  lv_obj_add_event_cb(btn_ok, [](lv_event_t *e) {
    play_beep(1200, 60);
    if (ready_banner) {
      lv_obj_delete_async(ready_banner);
      ready_banner = NULL;
    }
  }, LV_EVENT_CLICKED, NULL);

  lv_timer_create(close_ready_banner_timer_cb, 4000, NULL);
}

// --- Arduino Setup & Loop ---
void setup() {
  USBSerial.begin(115200);
  USBSerial.println("Starting Waveshare ESP32-S3 Touch AMOLED Watch...");

  // Register heap allocation failure hook so we never fail silently
  heap_caps_register_failed_alloc_callback(on_heap_alloc_failed);

  // Hardware reset FT3168 touch controller before I2C init
  pinMode(TP_RESET, OUTPUT);
  digitalWrite(TP_RESET, LOW);
  delay(25);
  digitalWrite(TP_RESET, HIGH);
  delay(120);
  pinMode(TP_INT, INPUT_PULLUP);

  Wire.begin(IIC_SDA, IIC_SCL);

  if (!gfx->begin()) {
    USBSerial.println("Display initialization failed!");
  }
  gfx->fillScreen(RGB565_BLACK);

  int touch_retry = 0;
  bool touch_ok = false;
  while (touch_retry < 5) {
    if (FT3168->begin()) {
      touch_ok = true;
      break;
    }
    USBSerial.println("Waiting for FT3168 touch...");
    digitalWrite(TP_RESET, LOW);
    delay(20);
    digitalWrite(TP_RESET, HIGH);
    delay(200);
    touch_retry++;
  }
  if (touch_ok) {
    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                   FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_ACTIVE);
    USBSerial.println("FT3168 touch initialized (ACTIVE mode)");
  } else {
    USBSerial.println("FT3168 begin() returned false, continuing with I2C polling");
  }

  pmu_available = power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  if (pmu_available) {
    power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    power.enableBattDetection();
    power.enableBattVoltageMeasure();
    power.enableVbusVoltageMeasure();
    power.enableSystemVoltageMeasure();
    // Waveshare board has no battery thermistor on TS pin; disable TS measurement for stable rail/charging
    power.disableTSPinMeasure();

    power.clearIrqStatus();
    power.enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ);

    USBSerial.println("AXP2101 PMU initialized");
  }

  rtc_available = rtc.begin(Wire, IIC_SDA, IIC_SCL);
  if (rtc_available) {
    USBSerial.println("PCF85063 RTC initialized");
  } else {
    USBSerial.println("PCF85063 RTC not found");
  }

  // Initialize ES8311 Audio Codec & Speaker Amplifier
  init_audio();

  prefs.begin("clock_app", false);
  current_tz_idx = prefs.getInt("tz_idx", 0);
  use_12h_format = prefs.getBool("use_12h", true);
  // Default Wi-Fi OFF until user enables or has saved preference
  wifi_power_on = prefs.getBool("wifi_power", false);
  ble_power_on = prefs.getBool("ble_power", true);
  alarm_enabled = prefs.getBool("alarm_en", false);
  alarm_hour = prefs.getInt("alarm_h", 7);
  alarm_min = prefs.getInt("alarm_m", 0);
  audio_muted = prefs.getBool("muted", false);
  display_brightness = prefs.getInt("bright", 208);
  selected_face_idx = prefs.getInt("face_idx", 0);

  // Restore saved clock baseline if RTC does not yet have a valid year
  int saved_yr = prefs.getInt("saved_yr", 2026);
  int saved_mo = prefs.getInt("saved_mo", 10);
  int saved_dy = prefs.getInt("saved_dy", 16);
  int saved_hr = prefs.getInt("saved_hr", 12);
  int saved_mn = prefs.getInt("saved_mn", 0);
  if (rtc_available) {
    RTC_DateTime dt = rtc.getDateTime();
    if (dt.getYear() < 2024 || dt.getYear() > 2099) {
      set_watch_datetime(saved_yr, saved_mo, saved_dy, saved_hr, saved_mn, 0);
    }
  } else {
    set_watch_datetime(saved_yr, saved_mo, saved_dy, saved_hr, saved_mn, 0);
  }

  set_amoled_brightness((uint8_t)display_brightness);

  // Initialize Bluetooth (BLE) BEFORE allocating LVGL DMA buffers so HCI controller gets contiguous Internal SRAM
  set_wifi_power(false);
  set_ble_power(ble_power_on);

  lv_init();
  lv_tick_set_cb(millis_cb);

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  // Single internal DMA buffer (synchronous flush in my_disp_flush only uses buf1)
  size_t buf_bytes = screenWidth * RENDER_BUF_HEIGHT * sizeof(lv_color_t);
  disp_draw_buf1 = (lv_color_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  disp_draw_buf2 = NULL;

  if (!disp_draw_buf1) {
    disp_draw_buf1 = (lv_color_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_8BIT);
  }

  // Keep standard malloc() in Internal SRAM so Bluedroid GATT/HCI and NVS flash writes
  // never touch PSRAM during cache-disabled or DMA sections.
  if (psramFound()) {
    USBSerial.printf("Octal PSRAM online (Free PSRAM: %u KB, Free Internal SRAM: %u KB)\n",
                     (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                     (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
  }

  if (disp_draw_buf1) {
    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf1, disp_draw_buf2, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touchpad_read);
    lv_display_add_event_cb(disp, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    create_ui();
    if (bar_bt_lbl) {
      lv_obj_set_style_text_color(bar_bt_lbl, (ble_power_on && ble_initialized) ? lv_color_hex(0x89B4FA) : lv_color_hex(0x6C7086), 0);
    }
    show_startup_ready_banner(touch_ok);
  }

  // Startup audio chime (non-blocking 60-90ms stack-buffered tones)
  play_beep(523, 60);
  play_beep(659, 60);
  play_beep(784, 90);

  last_touch_time = millis();
  update_computational_boundaries(0);
  char init_diag[384];
  format_boundary_report(init_diag, sizeof(init_diag));
  USBSerial.println("--- INITIAL COMPUTATIONAL BOUNDARIES ---");
  USBSerial.println(init_diag);
  USBSerial.println("READY: Setup completed successfully!");
}

// --- Weather Station Functions (Non-Blocking Background Task) ---
const char* get_weather_symbol(int code) {
  if (code == 0) return LV_SYMBOL_IMAGE;
  if (code >= 1 && code <= 3) return LV_SYMBOL_IMAGE;
  if (code >= 45 && code <= 48) return LV_SYMBOL_LIST;
  if (code >= 51 && code <= 67) return LV_SYMBOL_DOWN;
  if (code >= 71 && code <= 77) return LV_SYMBOL_BULLET;
  if (code >= 80 && code <= 82) return LV_SYMBOL_DOWN;
  if (code >= 95) return LV_SYMBOL_WARNING;
  return LV_SYMBOL_IMAGE;
}

void interpret_weather_code(int code, char* buf, size_t len) {
  switch(code) {
    case 0: snprintf(buf, len, "Clear / Sunny"); break;
    case 1: snprintf(buf, len, "Mainly Clear"); break;
    case 2: snprintf(buf, len, "Partly Cloudy"); break;
    case 3: snprintf(buf, len, "Overcast"); break;
    case 45: case 48: snprintf(buf, len, "Foggy"); break;
    case 51: case 53: case 55: snprintf(buf, len, "Drizzle"); break;
    case 61: case 63: case 65: snprintf(buf, len, "Rainy"); break;
    case 71: case 73: case 75: snprintf(buf, len, "Snowy"); break;
    case 80: case 81: case 82: snprintf(buf, len, "Rain Showers"); break;
    case 95: case 96: case 99: snprintf(buf, len, "Thunderstorm"); break;
    default: snprintf(buf, len, "Fair Weather"); break;
  }
}

static void weather_fetch_task(void *param) {
  HTTPClient http;
  http.setTimeout(3500);

  float lat = 0.0f, lon = 0.0f;
  String city_name = "Auto Location";

  // Step 1: IP Geolocation Lookup
  if (WiFi.status() == WL_CONNECTED) {
    http.begin("http://ip-api.com/json/");
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      int lat_idx = payload.indexOf("\"lat\":");
      int lon_idx = payload.indexOf("\"lon\":");
      int city_idx = payload.indexOf("\"city\":\"");
      if (lat_idx != -1 && lon_idx != -1) {
        lat = payload.substring(lat_idx + 6, payload.indexOf(",", lat_idx)).toFloat();
        lon = payload.substring(lon_idx + 6, payload.indexOf(",", lon_idx)).toFloat();
      }
      if (city_idx != -1) {
        int end_city = payload.indexOf("\"", city_idx + 8);
        if (end_city != -1) {
          city_name = payload.substring(city_idx + 8, end_city);
        }
      }
    }
    http.end();
  }

  if (lat == 0.0f && lon == 0.0f) {
    lat = 37.7749f;
    lon = -122.4194f;
    city_name = "San Francisco";
  }

  // Step 2: Open-Meteo Weather API Call
  char url[256];
  snprintf(url, sizeof(url),
    "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current=temperature_2m,relative_humidity_2m,weather_code,wind_speed_10m&daily=temperature_2m_max,temperature_2m_min&timezone=auto",
    lat, lon);

  int httpCode = -1;
  if (WiFi.status() == WL_CONNECTED) {
    http.begin(url);
    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();

      int temp_idx = payload.indexOf("\"temperature_2m\":");
      if (temp_idx != -1) {
        weather_data.temp_c = payload.substring(temp_idx + 17, payload.indexOf(",", temp_idx)).toFloat();
        weather_data.temp_f = (weather_data.temp_c * 9.0f / 5.0f) + 32.0f;
      }

      int hum_idx = payload.indexOf("\"relative_humidity_2m\":");
      if (hum_idx != -1) {
        weather_data.humidity = payload.substring(hum_idx + 23, payload.indexOf(",", hum_idx)).toInt();
      }

      int code_idx = payload.indexOf("\"weather_code\":");
      if (code_idx != -1) {
        weather_data.weather_code = payload.substring(code_idx + 15, payload.indexOf(",", code_idx)).toInt();
      }

      int wind_idx = payload.indexOf("\"wind_speed_10m\":");
      if (wind_idx != -1) {
        weather_data.wind_speed = payload.substring(wind_idx + 17, payload.indexOf("}", wind_idx)).toFloat();
      }

      int max_idx = payload.indexOf("\"temperature_2m_max\":[");
      if (max_idx != -1) {
        weather_data.temp_max_c = payload.substring(max_idx + 22, payload.indexOf(",", max_idx + 22)).toFloat();
        weather_data.temp_max_f = (weather_data.temp_max_c * 9.0f / 5.0f) + 32.0f;
      }

      int min_idx = payload.indexOf("\"temperature_2m_min\":[");
      if (min_idx != -1) {
        weather_data.temp_min_c = payload.substring(min_idx + 22, payload.indexOf(",", min_idx + 22)).toFloat();
        weather_data.temp_min_f = (weather_data.temp_min_c * 9.0f / 5.0f) + 32.0f;
      }

      snprintf(weather_data.city_str, sizeof(weather_data.city_str), "%s", city_name.c_str());
      interpret_weather_code(weather_data.weather_code, weather_data.condition_str, sizeof(weather_data.condition_str));

      weather_data.valid = true;
      weather_data.last_update_ms = millis();
      snprintf(weather_status_msg, sizeof(weather_status_msg), "Updated: Just now");
    } else {
      snprintf(weather_status_msg, sizeof(weather_status_msg), "Fetch Failed");
    }
    http.end();
  } else {
    snprintf(weather_status_msg, sizeof(weather_status_msg), "Offline Mode");
  }

  // Always update last_weather_fetch_time (even on failure) so UI loop never retries in a tight loop
  last_weather_fetch_time = millis();
  weather_ui_dirty = true;
  weather_fetch_in_progress = false;
  vTaskDelete(NULL);
}

void fetch_weather() {
  if (ble_power_on && ble_device_connected) {
    send_ble_message("REQ:WEATHER", true);
    snprintf(weather_status_msg, sizeof(weather_status_msg), "Requested via BT");
  } else {
    snprintf(weather_status_msg, sizeof(weather_status_msg), "Connect Android BT");
  }
  if (weather_status_lbl) lv_label_set_text(weather_status_lbl, weather_status_msg);
}

static void weather_refresh_btn_cb(lv_event_t * e) {
  play_beep(900, 50);
  fetch_weather();
}

void update_weather_display() {
  if (!face_container_weather) return;

  int year = 2026, month = 10, day = 18, hour = 12, min_val = 0, sec = 0;
  get_watch_datetime(year, month, day, hour, min_val, sec);

  char time_buf[32];
  if (use_12h_format) {
    int h12 = hour % 12;
    if (h12 == 0) h12 = 12;
    snprintf(time_buf, sizeof(time_buf), "%02d:%02d %s", h12, min_val, hour >= 12 ? "PM" : "AM");
  } else {
    snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02d", hour, min_val, sec);
  }
  if (weather_time_lbl) lv_label_set_text(weather_time_lbl, time_buf);

  char date_buf[64];
  snprintf(date_buf, sizeof(date_buf), "%04d-%02d-%02d • %s", year, month, day, weather_data.city_str);
  if (weather_date_lbl) lv_label_set_text(weather_date_lbl, date_buf);

  if (weather_data.valid) {
    char temp_c_buf[32];
    snprintf(temp_c_buf, sizeof(temp_c_buf), "%.1f°C", weather_data.temp_c);
    if (weather_temp_c_lbl) lv_label_set_text(weather_temp_c_lbl, temp_c_buf);

    char temp_f_buf[32];
    snprintf(temp_f_buf, sizeof(temp_f_buf), "(%.1f°F)", weather_data.temp_f);
    if (weather_temp_f_lbl) lv_label_set_text(weather_temp_f_lbl, temp_f_buf);

    if (weather_cond_lbl) lv_label_set_text(weather_cond_lbl, weather_data.condition_str);

    char det_buf[96];
    snprintf(det_buf, sizeof(det_buf), "H: %.0f°C (%.0f°F)  L: %.0f°C (%.0f°F)\nHum: %d%%   Wind: %.1f km/h",
             weather_data.temp_max_c, weather_data.temp_max_f,
             weather_data.temp_min_c, weather_data.temp_min_f,
             weather_data.humidity, weather_data.wind_speed);
    if (weather_details_lbl) lv_label_set_text(weather_details_lbl, det_buf);

    if (weather_status_lbl) {
      uint32_t ago_sec = (millis() - weather_data.last_update_ms) / 1000;
      char st_buf[64];
      if (ago_sec < 60) {
        snprintf(st_buf, sizeof(st_buf), "Updated: Just now");
      } else {
        snprintf(st_buf, sizeof(st_buf), "Updated: %lu mins ago", (unsigned long)(ago_sec / 60));
      }
      lv_label_set_text(weather_status_lbl, st_buf);
    }
  } else {
    if (weather_temp_c_lbl) lv_label_set_text(weather_temp_c_lbl, "--°C");
    if (weather_temp_f_lbl) lv_label_set_text(weather_temp_f_lbl, "(--°F)");
    if (weather_cond_lbl) lv_label_set_text(weather_cond_lbl, "No Weather Data");
    if (weather_details_lbl) lv_label_set_text(weather_details_lbl, "Send WEATHER:<c>,<cond>,<city> via BT");
    if (weather_status_lbl) lv_label_set_text(weather_status_lbl, weather_status_msg);
  }
}

void loop() {
  uint32_t loop_t0 = millis();

  if (pmu_available) {
    power.getIrqStatus();
    if (power.isPekeyShortPressIrq()) {
      power.clearIrqStatus();
      if (is_screen_sleeping) {
        wake_screen();
      } else {
        sleep_screen();
      }
    }
  }

  if (!is_screen_sleeping && auto_sleep_timeout_ms > 0 && !ble_device_connected) {
    if (millis() - last_touch_time > auto_sleep_timeout_ms) {
      sleep_screen();
    }
  }

  // Non-blocking status updates (Wi-Fi, NTP, BLE, Weather UI flag)
  update_wifi_scan_results();
  update_wifi_connect_status();
  update_ntp_sync_status();
  update_ble_status();

  if (weather_ui_dirty) {
    weather_ui_dirty = false;
    update_weather_display();
  }

  lv_timer_handler();

  if (!is_screen_sleeping) {
    update_clock_display();
    update_tools_display();
    update_pmu_display();
  }

  update_computational_boundaries(millis() - loop_t0);
  delay(5);
}


