// Composite video (CVBS) output, see tv_out.h.
//
// Signal: 240p, 10 MHz sample clock (100 ns per sample), one byte per sample;
// data bit 2 drives G5, bit 1 G6, bit 0 G3. Levels into the TV's 75 R input:
// 0 = sync 0 V, 2 = black 0.30 V, 3..6 = greys, 7 = white 1.07 V. In mono
// mode G3 is not driven and white is 6 (G5 + G6).
// Horizontal and vertical sync pulses are part of the data, so a line that is
// not refilled in time only shows as a one-line glitch.
//
// Output: LCD_CAM in i8080 mode (data phase only, always on) fed by a
// circular ring of GDMA descriptors, one line per buffer. The GDMA "descriptor
// done" interrupt (IRAM, level 3) renders the next lines from the emulator's
// frame buffer. Based on the driver of the Cardputer ADV composite test
// pattern generator.

#include "tv_out.h"

#include <M5Cardputer.h>
#include <Preferences.h>
#include <sdkconfig.h>
#include <string.h>

#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_intr_alloc.h>
#include <esp_private/gdma.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <soc/gpio_sig_map.h>
#include <soc/lcd_cam_reg.h>
#include <soc/soc.h>

#if __has_include(<esp_private/periph_ctrl.h>)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#if __has_include(<soc/interrupts.h>)
#include <soc/interrupts.h>
#else
#include <soc/periph_defs.h>
#endif
#if defined(LCD_CAM_LCD_UPDATE)
#define TVOUT_LCD_UPDATE_BIT LCD_CAM_LCD_UPDATE
#else
#define TVOUT_LCD_UPDATE_BIT LCD_CAM_LCD_UPDATE_REG
#endif

#ifndef TVOUT_PIN_HI
#define TVOUT_PIN_HI 5   // bit 2, 270 R (220 R for mono only)
#endif
#ifndef TVOUT_PIN_MID
#define TVOUT_PIN_MID 6  // bit 1, 560 R
#endif
#ifndef TVOUT_PIN_LO
#define TVOUT_PIN_LO 3   // bit 0, 1.1 k, 6-grey mode only
#endif
#ifndef TVOUT_CORE
#define TVOUT_CORE 0     // core that runs the interrupt
#endif

namespace {

constexpr int kBufs = 4;           // line buffers in the DMA ring
constexpr int kBufBytes = 640;     // longest line
constexpr int kActiveLines = 240;
constexpr int kActiveSamples = 512;
constexpr int kVsyncLines = 3;
constexpr uint8_t kBlack = 2;
constexpr uint32_t kBlack4 = 0x02020202u;

struct Std {
  uint16_t len;       // samples per line
  uint16_t lines;     // lines per frame
  uint16_t half;      // half line, for the vertical sync pulses
  uint16_t broadLow;  // low part of a vertical sync half line
  uint16_t x0;        // first sample of the 512-sample active area (multiple of 4)
  uint16_t top;       // first line of the 240-line active area
};
// Same geometry as the Zimodem terminal output.
const Std kStd[2] = {
  {640, 312, 320, 273, 108, 44},  // PAL 240p: 64.0 us per line, 50.08 Hz
  {636, 262, 318, 271, 104, 20},  // NTSC 240p: 63.6 us per line, 60.02 Hz
};
constexpr int kHsync = 47;  // 4.7 us

enum : uint8_t { T_BLANK = 0, T_BROAD, T_ACTIVE };

struct Desc {
  volatile uint32_t dw0;  // size[11:0], length[23:12], owner[31]
  const uint8_t* buf;
  Desc* next;
};

// GDMA registers, one block of 0xC0 bytes per channel (ESP32-S3 TRM).
constexpr uint32_t kGdmaBase = 0x6003F000u;
constexpr uint32_t kGdmaStride = 0xC0u;

// ---- used by the interrupt: plain RAM only ----
Std s_std;
uint8_t* s_buf[kBufs];
uint8_t s_bufType[kBufs];
uint32_t* s_tplBlank;
uint32_t* s_tplBroad;
uint16_t s_lut[2][256];  // two samples per source pixel, by line parity
const uint8_t* const* volatile s_src;
volatile int s_srcW, s_srcH;
int s_line;
Desc* s_desc;
uint32_t s_regStatus, s_regClear, s_regDesc;
uint32_t s_produced, s_dmaLines, s_lastDesc;

// ---- control ----
uint8_t s_luma[256];
int s_mode = TVOUT_OFF;
bool s_supported;
volatile bool s_lcdDirty;
gdma_channel_handle_t s_chan;
intr_handle_t s_intr;
int8_t s_pins[3] = {-1, -1, -1};
SemaphoreHandle_t s_startDone;
volatile bool s_startOk;

const char* const kNames[TVOUT_MODES] = {
  "", "TV out: PAL, 6 greys", "TV out: NTSC, 6 greys",
  "TV out: PAL, mono", "TV out: NTSC, mono"};

inline bool isMono(int m) { return m == TVOUT_PAL_MONO || m == TVOUT_NTSC_MONO; }
inline bool isNtsc(int m) { return m == TVOUT_NTSC_GREY || m == TVOUT_NTSC_MONO; }

void IRAM_ATTR copyWords(uint32_t* d, const uint32_t* s, int n) {
  while (n--) *d++ = *s++;
}
void IRAM_ATTR fillWords(uint32_t* d, uint32_t v, int n) {
  while (n--) *d++ = v;
}

// Renders the next line of the frame into buffer b.
void IRAM_ATTR renderLine(int b) {
  uint8_t* d = s_buf[b];
  const int line = s_line;
  s_line = (line + 1 < s_std.lines) ? line + 1 : 0;

  if (line < kVsyncLines) {
    if (s_bufType[b] != T_BROAD) {
      copyWords(reinterpret_cast<uint32_t*>(d), s_tplBroad, kBufBytes / 4);
      s_bufType[b] = T_BROAD;
    }
    return;
  }
  if (s_bufType[b] == T_BROAD) {
    copyWords(reinterpret_cast<uint32_t*>(d), s_tplBlank, kBufBytes / 4);
    s_bufType[b] = T_BLANK;
  }

  const uint8_t* const* src = s_src;
  const int h = s_srcH;
  const int y = line - s_std.top - ((kActiveLines - h) >> 1);
  if (!src || static_cast<unsigned>(y) >= static_cast<unsigned>(h)) {
    if (s_bufType[b] == T_ACTIVE) {
      fillWords(reinterpret_cast<uint32_t*>(d + s_std.x0), kBlack4, kActiveSamples / 4);
      s_bufType[b] = T_BLANK;
    }
    return;
  }

  const int w = s_srcW;
  const uint8_t* p = src[y];
  uint16_t* o = reinterpret_cast<uint16_t*>(d + s_std.x0 + ((kActiveSamples - 2 * w) >> 1));
  const uint16_t* lut = s_lut[line & 1];
  int x = 0;
  for (; x + 4 <= w; x += 4) {
    o[x] = lut[p[x]];
    o[x + 1] = lut[p[x + 1]];
    o[x + 2] = lut[p[x + 2]];
    o[x + 3] = lut[p[x + 3]];
  }
  for (; x < w; ++x) o[x] = lut[p[x]];
  s_bufType[b] = T_ACTIVE;
}

// GDMA "descriptor done": refill the buffers the DMA has finished with.
void IRAM_ATTR isr(void*) {
  REG_WRITE(s_regClear, REG_READ(s_regStatus));
  const uint32_t offset = REG_READ(s_regDesc) - reinterpret_cast<uint32_t>(s_desc);
  if (offset >= sizeof(Desc) * kBufs) return;
  const uint32_t current = offset / sizeof(Desc);
  s_dmaLines += (current + kBufs - s_lastDesc) % kBufs;
  s_lastDesc = current;
  // Line m lives in buffer m % N; it can be rendered once the previous
  // occupant (line m - N) is at least two lines behind the DMA.
  while (s_produced + 2 <= s_dmaLines + kBufs) {
    renderLine(s_produced % kBufs);
    ++s_produced;
  }
}

// ---- level tables ----

void lutEntry(int i) {
  const int l = s_luma[i];
  uint16_t even, odd;
  if (isMono(s_mode)) {
    // 2x2 ordered dither: 0..4 of the 4 samples are white.
    const int q = (l * 5) >> 8;
    const uint16_t W = 6, B = kBlack;
    even = (q >= 1 ? W : B) | (q >= 3 ? W : B) << 8;
    odd = (q >= 4 ? W : B) | (q >= 2 ? W : B) << 8;
  } else {
    const uint16_t v = kBlack + ((l * 6) >> 8);  // 2..7
    even = odd = v | v << 8;
  }
  s_lut[0][i] = even;
  s_lut[1][i] = odd;
}

void buildLut() {
  for (int i = 0; i < 256; ++i) lutEntry(i);
}

// ---- hardware ----

void stopHw() {
  if (s_chan || s_intr) REG_CLR_BIT(LCD_CAM_LCD_USER_REG, LCD_CAM_LCD_START);
  if (s_intr) {
    esp_intr_free(s_intr);
    s_intr = nullptr;
  }
  if (s_chan) {
    gdma_stop(s_chan);
    gdma_disconnect(s_chan);
    gdma_del_channel(s_chan);
    s_chan = nullptr;
  }
  for (int i = 0; i < 3; ++i) {
    if (s_pins[i] >= 0) gpio_reset_pin(static_cast<gpio_num_t>(s_pins[i]));
    s_pins[i] = -1;
  }
  for (int i = 0; i < kBufs; ++i) {
    heap_caps_free(s_buf[i]);
    s_buf[i] = nullptr;
  }
  heap_caps_free(s_desc);
  heap_caps_free(s_tplBlank);
  heap_caps_free(s_tplBroad);
  s_desc = nullptr;
  s_tplBlank = s_tplBroad = nullptr;
}

// Runs on TVOUT_CORE: interrupts are allocated on the calling core.
bool startOnCore() {
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  // PCLK = PLL_D2 240 MHz / 24 = 10 MHz, output clock = module clock.
  REG_WRITE(LCD_CAM_LCD_CLOCK_REG, LCD_CAM_CLK_EN | (2u << LCD_CAM_LCD_CLK_SEL_S)
                                       | (24u << LCD_CAM_LCD_CLKM_DIV_NUM_S)
                                       | LCD_CAM_LCD_CLK_EQU_SYSCLK);
  REG_WRITE(LCD_CAM_LCD_RGB_YUV_REG, 0);
  REG_WRITE(LCD_CAM_LCD_CTRL_REG, 0);  // i8080 mode
  REG_WRITE(LCD_CAM_LCD_CTRL1_REG, 0);
  REG_WRITE(LCD_CAM_LCD_CTRL2_REG, 0);
  REG_WRITE(LCD_CAM_LCD_DLY_MODE_REG, 0);
  REG_WRITE(LCD_CAM_LCD_DATA_DOUT_MODE_REG, 0);
  REG_CLR_BIT(LCD_CAM_LCD_MISC_REG, LCD_CAM_LCD_NEXT_FRAME_EN | LCD_CAM_LCD_BK_EN
                                        | LCD_CAM_LCD_CD_IDLE_EDGE | LCD_CAM_LCD_CD_CMD_SET
                                        | LCD_CAM_LCD_CD_DUMMY_SET | LCD_CAM_LCD_CD_DATA_SET);
  REG_WRITE(LCD_CAM_LCD_USER_REG, LCD_CAM_LCD_DOUT | LCD_CAM_LCD_ALWAYS_OUT_EN);
  REG_WRITE(LCD_CAM_LC_DMA_INT_ENA_REG, 0);
  REG_WRITE(LCD_CAM_LC_DMA_INT_CLR_REG, 0xFFFFFFFFu);

  for (int bit = 0; bit < 3; ++bit) {
    const int pin = s_pins[bit];
    if (pin < 0) continue;
    gpio_reset_pin(static_cast<gpio_num_t>(pin));
    gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT);
    gpio_set_drive_capability(static_cast<gpio_num_t>(pin), GPIO_DRIVE_CAP_3);
    esp_rom_gpio_connect_out_signal(pin, LCD_DATA_OUT0_IDX + bit, false, false);
  }

  gdma_channel_alloc_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.direction = GDMA_CHANNEL_DIRECTION_TX;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (gdma_new_channel(&cfg, &s_chan) != ESP_OK) return false;
#pragma GCC diagnostic pop
  gdma_connect(s_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
  gdma_strategy_config_t strategy;
  memset(&strategy, 0, sizeof(strategy));
  gdma_apply_strategy(s_chan, &strategy);
  int id = -1;
  if (gdma_get_channel_id(s_chan, &id) != ESP_OK || id < 0) return false;
  const uint32_t base = kGdmaBase + kGdmaStride * static_cast<uint32_t>(id);
  s_regStatus = base + 0x6Cu;
  s_regClear = base + 0x74u;
  s_regDesc = base + 0x90u;  // address of the current descriptor
  REG_WRITE(base + 0x70u, 0);
  REG_WRITE(s_regClear, 0xFFFFFFFFu);
  if (esp_intr_alloc(ETS_DMA_OUT_CH0_INTR_SOURCE + id, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
                     isr, nullptr, &s_intr) != ESP_OK) {
    return false;
  }
  REG_WRITE(base + 0x70u, 1u);  // "out done" interrupt

  REG_SET_BIT(LCD_CAM_LCD_USER_REG, LCD_CAM_LCD_RESET);
  REG_SET_BIT(LCD_CAM_LCD_MISC_REG, LCD_CAM_LCD_AFIFO_RESET);
  gdma_reset(s_chan);
  gdma_start(s_chan, reinterpret_cast<intptr_t>(s_desc));
  esp_rom_delay_us(2);
  REG_SET_BIT(LCD_CAM_LCD_USER_REG, TVOUT_LCD_UPDATE_BIT);
  REG_SET_BIT(LCD_CAM_LCD_USER_REG, LCD_CAM_LCD_START);
  return true;
}

void startTask(void*) {
  s_startOk = startOnCore();
  xSemaphoreGive(s_startDone);
  vTaskDelete(nullptr);
}

bool startHw() {
  s_std = kStd[isNtsc(s_mode) ? 1 : 0];
  const uint32_t dma = MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  s_desc = static_cast<Desc*>(heap_caps_aligned_alloc(4, sizeof(Desc) * kBufs, dma));
  s_tplBlank = static_cast<uint32_t*>(heap_caps_malloc(kBufBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT));
  s_tplBroad = static_cast<uint32_t*>(heap_caps_malloc(kBufBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT));
  bool ok = s_desc && s_tplBlank && s_tplBroad;
  for (int i = 0; i < kBufs; ++i) {
    s_buf[i] = static_cast<uint8_t*>(heap_caps_aligned_alloc(4, kBufBytes, dma));
    ok = ok && s_buf[i];
  }
  if (!ok) return false;

  // Line templates: normal line (sync + black) and vertical sync line.
  uint8_t* blank = reinterpret_cast<uint8_t*>(s_tplBlank);
  uint8_t* broad = reinterpret_cast<uint8_t*>(s_tplBroad);
  memset(blank, kBlack, kBufBytes);
  memset(blank, 0, kHsync);
  memset(broad, kBlack, kBufBytes);
  memset(broad, 0, s_std.broadLow);
  memset(broad + s_std.half, 0, s_std.broadLow);

  s_line = 0;
  for (int i = 0; i < kBufs; ++i) {
    memcpy(s_buf[i], blank, kBufBytes);
    s_bufType[i] = T_BLANK;
    renderLine(i);
    s_desc[i].dw0 = (1u << 31) | (static_cast<uint32_t>(s_std.len) << 12) | kBufBytes;
    s_desc[i].buf = s_buf[i];
    s_desc[i].next = &s_desc[(i + 1) % kBufs];
  }
  s_produced = kBufs;
  s_dmaLines = 0;
  s_lastDesc = 0;

  s_pins[2] = TVOUT_PIN_HI;
  s_pins[1] = TVOUT_PIN_MID;
  s_pins[0] = isMono(s_mode) ? -1 : TVOUT_PIN_LO;

  s_startDone = xSemaphoreCreateBinary();
  if (!s_startDone) return false;
  s_startOk = false;
  if (xTaskCreatePinnedToCore(startTask, "tvout", 3072, nullptr, 20, nullptr, TVOUT_CORE) == pdPASS) {
    xSemaphoreTake(s_startDone, portMAX_DELAY);
  }
  vSemaphoreDelete(s_startDone);
  s_startDone = nullptr;
  return s_startOk;
}

void setMode(int mode, bool save) {
  stopHw();
  s_mode = mode;
  buildLut();
  if (s_mode != TVOUT_OFF && !startHw()) {
    stopHw();
    s_mode = TVOUT_OFF;
  }
  s_lcdDirty = true;
  if (save) {
    Preferences prefs;
    prefs.begin("cardputer_emu", false);
    prefs.putUChar("tv_mode", static_cast<uint8_t>(s_mode));
    prefs.end();
  }
}

}  // namespace

extern "C" {

void tvout_init(void) {
  s_supported = true;
  Preferences prefs;
  prefs.begin("cardputer_emu", true);
  int mode = prefs.getUChar("tv_mode", TVOUT_OFF);
  prefs.end();
  if (mode > TVOUT_OFF && mode < TVOUT_MODES) setMode(mode, false);
}

int tvout_mode(void) { return s_mode; }

void tvout_cycle_mode(void) {
  if (s_supported) setMode((s_mode + 1) % TVOUT_MODES, true);
}

void tvout_set_source8(const uint8_t* const* lines, int width, int height) {
  if (width > kActiveSamples / 2) width = kActiveSamples / 2;
  width &= ~1;  // keeps the 16-bit stores aligned
  if (height > kActiveLines) height = kActiveLines;
  s_srcW = width;
  s_srcH = height;
  s_src = lines;
}

void tvout_set_luma(int index, uint8_t luma) {
  s_luma[index & 255] = luma;
  lutEntry(index & 255);
}

int tvout_lcd_hook(void) {
  if (s_lcdDirty) {
    s_lcdDirty = false;
    auto& lcd = M5Cardputer.Display;
    lcd.fillScreen(TFT_BLACK);
    if (s_mode != TVOUT_OFF) {
      lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      lcd.drawCenterString(kNames[s_mode], lcd.width() / 2, lcd.height() / 2 - 12);
      lcd.drawCenterString("T: change mode", lcd.width() / 2, lcd.height() / 2 + 4);
    }
  }
  return s_mode != TVOUT_OFF;
}

}  // extern "C"
