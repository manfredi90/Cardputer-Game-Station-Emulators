// Composite video (CVBS) output, see tv_out.h.
//
// Signal: 240p, one byte per sample; data bit 2 drives G5, bit 1 G6, bit 0
// G3. Levels into the TV's 75 R input: 0 = sync 0 V, 2 = black 0.30 V,
// 3..6 = greys, 7 = white 1.07 V. In mono mode G3 is not driven and white is
// 6 (G5 + G6). Horizontal and vertical sync pulses are part of the data, so a
// line that is not refilled in time only shows as a one-line glitch.
//
// Sample clock: one of five clocks with an integer divider (fractional
// dividers make the pixels uneven), from the 160 or 240 MHz PLL. The clock
// sets the picture width; the line and sync timings are computed for it.
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
#include <stdio.h>
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
constexpr int kBufBytes = 736;     // longest line: 11.43 MHz x 64 us = 732 samples
constexpr int kMaxActive = 512;
constexpr int kActiveLines = 240;
constexpr int kVsyncLines = 3;
constexpr uint8_t kBlack = 2;
constexpr uint32_t kBlack4 = 0x02020202u;

// Sample clocks with an integer divider, widest picture first.
struct Clock {
  uint8_t sel;  // LCD_CAM clock source: 2 = PLL 240 MHz, 3 = PLL 160 MHz
  uint8_t div;
};
const Clock kClocks[] = {{3, 18}, {2, 26}, {3, 16}, {2, 22}, {3, 14}};
constexpr int kNumClocks = sizeof(kClocks) / sizeof(kClocks[0]);
constexpr int kClockDef = 2;  // 10.00 MHz
inline uint32_t clockHz(int i) { return (kClocks[i].sel == 2 ? 240000000u : 160000000u) / kClocks[i].div; }

// Line and frame geometry for one standard and clock.
struct Geo {
  uint16_t len;       // samples per line (even)
  uint16_t lines;     // lines per frame
  uint16_t half;      // half line, for the vertical sync pulses
  uint16_t broadLow;  // low part of a vertical sync half line
  uint16_t hsync;     // horizontal sync pulse
  uint16_t x0;        // first sample of the active area (even)
  uint16_t active;    // samples in the active area (multiple of 4, <= 512)
  uint16_t top;       // first line of the 240-line active area
  uint16_t clr0;      // word-aligned span that covers the active area
  uint16_t clrWords;
};
constexpr int kTopPal = 44, kTopNtsc = 20, kTopMin = 8;

// Calibration range for one standard and clock, in steps (2 samples, 1 line).
struct Range {
  int xmin, xmax, ymin, ymax;
};

Geo makeGeo(bool ntsc, int clk, int xoff, int yoff, Range* range) {
  const float f = clockHz(clk) / 1e6f;  // samples per microsecond
  Geo g;
  g.len = static_cast<uint16_t>(static_cast<int>((ntsc ? 63.556f : 64.0f) * f + 0.5f) & ~1);
  g.lines = ntsc ? 262 : 312;
  g.half = g.len / 2;
  g.hsync = static_cast<uint16_t>(4.7f * f + 0.5f);
  g.broadLow = g.half - g.hsync;
  int active = static_cast<int>(52.0f * f) & ~3;  // visible part of a line
  if (active > kMaxActive) active = kMaxActive;
  g.active = static_cast<uint16_t>(active);
  // Centre of the visible part: 36.4 us (PAL) / 36.0 us (NTSC) after the
  // start of sync.
  const int base = (static_cast<int>((ntsc ? 36.0f : 36.4f) * f + 0.5f) - active / 2) & ~1;
  const int xlo = g.hsync + static_cast<int>(1.5f * f + 0.5f);       // after sync
  const int xhi = g.len - static_cast<int>(0.5f * f + 0.5f) - active;  // before sync
  Range r;
  r.xmin = 0;
  while (base + 2 * (r.xmin - 1) >= xlo) --r.xmin;
  r.xmax = 0;
  while (base + 2 * (r.xmax + 1) <= xhi) ++r.xmax;
  const int topDef = ntsc ? kTopNtsc : kTopPal;
  r.ymin = kTopMin - topDef;
  r.ymax = g.lines - kActiveLines - topDef;
  if (xoff < r.xmin) xoff = r.xmin;
  if (xoff > r.xmax) xoff = r.xmax;
  if (yoff < r.ymin) yoff = r.ymin;
  if (yoff > r.ymax) yoff = r.ymax;
  g.x0 = static_cast<uint16_t>(base + 2 * xoff);
  g.top = static_cast<uint16_t>(topDef + yoff);
  g.clr0 = g.x0 & ~3;
  g.clrWords = static_cast<uint16_t>((g.x0 + g.active - g.clr0 + 3) / 4);
  if (range) *range = r;
  return g;
}

enum : uint8_t { T_BLANK = 0, T_BROAD, T_ACTIVE };

// A source frame, ready for the interrupt: cropped to fit, base adjusted.
struct Src {
  const uint8_t* base;  // first pixel shown
  int pitch;            // bytes per source line
  int w, h;             // pixels shown (after the x step) and lines
  int ox;               // first sample, from the start of the active area (even)
  int ty;               // first active line used
  uint8_t wide;         // 2-byte pixels
  uint8_t k;            // samples per pixel, 1..3
  uint8_t v;            // TV lines per source line, 1..2
  uint8_t step;         // source pixels per shown pixel, 1..2
  uint32_t gen;         // geometry generation, to clear stale borders
};

struct Desc {
  volatile uint32_t dw0;  // size[11:0], length[23:12], owner[31]
  const uint8_t* buf;
  Desc* next;
};

// GDMA registers, one block of 0xC0 bytes per channel (ESP32-S3 TRM).
constexpr uint32_t kGdmaBase = 0x6003F000u;
constexpr uint32_t kGdmaStride = 0xC0u;

// ---- used by the interrupt: plain RAM only ----
Geo s_g;
uint8_t* s_buf[kBufs];
uint8_t s_bufType[kBufs];
uint32_t s_bufGen[kBufs];
uint32_t* s_tplBlank;
uint32_t* s_tplBroad;
uint16_t s_lut[2][256];  // two samples per source pixel, by line parity
uint8_t s_lumHi[256], s_lumLo[256];  // brightness of the bytes of a 16-bit pixel
Src s_srcSlot[2];
const Src* volatile s_src;  // switched between the two slots
volatile bool s_test;       // calibration picture instead of the game
uint8_t s_white = 7;
int s_line;
Desc* s_desc;
uint32_t s_regStatus, s_regClear, s_regDesc;
uint32_t s_produced, s_dmaLines, s_lastDesc;

// ---- settings ----
bool s_on;
int s_std = TVOUT_PAL_GREY;  // standard and picture when on
int s_clk = kClockDef;
int s_xoff, s_yoff;
int s_mode = TVOUT_OFF;      // what is running: TVOUT_OFF or s_std

// ---- control ----
uint8_t s_luma[256];
uint8_t s_srcFormat = 0xFF;
tvout_source_t s_in;  // last source passed in (reapplied on geometry changes)
bool s_inValid;
portMUX_TYPE s_srcMux = portMUX_INITIALIZER_UNLOCKED;
bool s_supported;
volatile bool s_lcdDirty;
volatile bool s_menuOpen;
gdma_channel_handle_t s_chan;
intr_handle_t s_intr;
int8_t s_pins[3] = {-1, -1, -1};
SemaphoreHandle_t s_startDone;
volatile bool s_startOk;

inline bool isMono(int m) { return m == TVOUT_PAL_MONO || m == TVOUT_NTSC_MONO; }
inline bool isNtsc(int m) { return m == TVOUT_NTSC_GREY || m == TVOUT_NTSC_MONO; }
inline int makeStd(bool ntsc, bool mono) {
  return mono ? (ntsc ? TVOUT_NTSC_MONO : TVOUT_PAL_MONO) : (ntsc ? TVOUT_NTSC_GREY : TVOUT_PAL_GREY);
}

void IRAM_ATTR copyWords(uint32_t* d, const uint32_t* s, int n) {
  while (n--) *d++ = *s++;
}
void IRAM_ATTR fillWords(uint32_t* d, uint32_t v, int n) {
  while (n--) *d++ = v;
}
void IRAM_ATTR fillBytes(uint8_t* d, uint8_t v, int n) {
  while (n--) *d++ = v;
}

// Calibration picture: white frame on the edges of the active area, grid,
// centre cross and a grey ramp (black/white bars in mono).
void IRAM_ATTR renderTest(uint8_t* d, int y) {
  const int a = s_g.active;
  const uint8_t w = s_white;
  const uint8_t grid = w == 7 ? 4 : w;
  uint8_t* o = d + s_g.x0;
  if (y < 2 || y >= kActiveLines - 2 || y == kActiveLines / 2 - 1 || y == kActiveLines / 2) {
    fillBytes(o, w, a);
    return;
  }
  fillBytes(o, kBlack, a);
  if (y % 40 == 0) {
    fillBytes(o, grid, a);
  } else {
    for (int x = (a / 2) % 64; x < a; x += 64) o[x] = o[x + 1] = grid;
  }
  if (y >= 150 && y < 190) {  // ramp over the middle 3/4
    const int bw = (a * 3 / 4) / 6, x0 = (a - 6 * bw) / 2;
    for (int i = 0; i < 6; ++i) {
      const uint8_t v = w == 7 ? static_cast<uint8_t>(kBlack + i) : (i & 1) ? w : kBlack;
      fillBytes(o + x0 + i * bw, v, bw);
    }
  }
  o[a / 2 - 1] = o[a / 2] = w;
  o[0] = o[1] = o[a - 2] = o[a - 1] = w;
}

// Renders the next line of the frame into buffer b.
void IRAM_ATTR renderLine(int b) {
  uint8_t* d = s_buf[b];
  const int line = s_line;
  s_line = (line + 1 < s_g.lines) ? line + 1 : 0;

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

  const int yt = line - s_g.top;
  if (s_test) {
    if (static_cast<unsigned>(yt) < static_cast<unsigned>(kActiveLines)) {
      renderTest(d, yt);
      s_bufType[b] = T_ACTIVE;
      s_bufGen[b] = 0;
    } else if (s_bufType[b] == T_ACTIVE) {
      fillWords(reinterpret_cast<uint32_t*>(d + s_g.clr0), kBlack4, s_g.clrWords);
      s_bufType[b] = T_BLANK;
    }
    return;
  }

  const Src* src = s_src;
  int y = src ? yt - src->ty : -1;
  if (src && src->v == 2) y >>= 1;
  if (!src || y < 0 || y >= src->h) {
    if (s_bufType[b] == T_ACTIVE) {
      fillWords(reinterpret_cast<uint32_t*>(d + s_g.clr0), kBlack4, s_g.clrWords);
      s_bufType[b] = T_BLANK;
    }
    return;
  }
  if (s_bufType[b] == T_ACTIVE && s_bufGen[b] != src->gen) {
    fillWords(reinterpret_cast<uint32_t*>(d + s_g.clr0), kBlack4, s_g.clrWords);
  }
  s_bufGen[b] = src->gen;
  s_bufType[b] = T_ACTIVE;

  const uint16_t* lut = s_lut[line & 1];
  const uint8_t* p = src->base + y * src->pitch;
  const uint16_t* p16 = reinterpret_cast<const uint16_t*>(p);
  uint8_t* o = d + s_g.x0 + src->ox;
  const int w = src->w;
  const int step = src->step;

// Pixel x shown -> two samples (left in the low byte).
#define TV_PIX8(x) lut[p[(x)*step]]
#define TV_PIX16(x) lut[s_lumHi[p16[(x)*step] >> 8] + s_lumLo[p16[(x)*step] & 0xFF]]
#define TV_ROW(PIX)                                                          \
  switch (src->k) {                                                          \
    case 2: {                                                                \
      uint16_t* o16 = reinterpret_cast<uint16_t*>(o);                        \
      for (int x = 0; x < w; ++x) o16[x] = PIX(x);                           \
      break;                                                                 \
    }                                                                        \
    case 3:                                                                  \
      for (int x = 0; x < w; ++x, o += 3) {                                  \
        const uint16_t q = PIX(x);                                           \
        o[0] = q; o[1] = q >> 8; o[2] = q;                                   \
      }                                                                      \
      break;                                                                 \
    default:                                                                 \
      for (int x = 0; x < w; ++x) {                                          \
        const uint16_t q = PIX(x);                                           \
        o[x] = (x & 1) ? (q >> 8) : q;                                       \
      }                                                                      \
  }
  if (src->wide) {
    TV_ROW(TV_PIX16)
  } else {
    TV_ROW(TV_PIX8)
  }
#undef TV_ROW
#undef TV_PIX16
#undef TV_PIX8
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
  if (isMono(s_std)) {
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

// ---- source ----

// Fits the last source to the current geometry and hands it to the interrupt.
void applySource() {
  static uint32_t gen = 1;
  if (!s_inValid) {
    s_src = nullptr;
    return;
  }
  const tvout_source_t* in = &s_in;
  if (in->format != s_srcFormat) {
    s_srcFormat = in->format;
    // 16-bit pixels: brightness = table(high byte) + table(low byte), each
    // part of R, G and B being a linear term. The level table is then
    // indexed by brightness.
    for (int i = 0; i < 256; ++i) {
      int hi = 0, lo = 0;  // x 256
      switch (in->format) {
        case TVOUT_FMT_RGB565:          // hi RRRRRGGG, lo GGGBBBBB
        case TVOUT_FMT_RGB565_SWAPPED:  // bytes exchanged in memory
          hi = 77 * ((i >> 3) * 255 / 31) + 150 * (((i & 7) << 3) * 255 / 63);
          lo = 150 * ((i >> 5) * 255 / 63) + 29 * ((i & 31) * 255 / 31);
          break;
        case TVOUT_FMT_BGR444:  // hi 0000BBBB, lo GGGGRRRR
          hi = 29 * ((i & 15) * 17);
          lo = 150 * ((i >> 4) * 17) + 77 * ((i & 15) * 17);
          break;
      }
      if (in->format == TVOUT_FMT_RGB565_SWAPPED) {
        s_lumLo[i] = hi >> 8;
        s_lumHi[i] = lo >> 8;
      } else {
        s_lumHi[i] = hi >> 8;
        s_lumLo[i] = lo >> 8;
      }
    }
    if (in->format != TVOUT_FMT_INDEX8) {
      for (int i = 0; i < 256; ++i) s_luma[i] = i;
      buildLut();
    }
  }

  portENTER_CRITICAL(&s_srcMux);
  const Src* cur = s_src;
  Src& n = s_srcSlot[cur == &s_srcSlot[0] ? 1 : 0];
  const int bpp = in->format == TVOUT_FMT_INDEX8 ? 1 : 2;
  n.wide = bpp == 2;
  n.k = in->xscale < 1 ? 1 : in->xscale > 3 ? 3 : in->xscale;
  n.v = in->yscale == 2 ? 2 : 1;
  n.step = in->xstep == 2 ? 2 : 1;
  n.pitch = in->pitch;
  // Crop what does not fit (centred), then centre what is left.
  const int active = s_g.active;
  int w = in->width / n.step;
  int h = in->height;
  const int maxW = active / n.k, maxH = kActiveLines / n.v;
  const int cx = w > maxW ? (w - maxW) / 2 : 0;
  const int cy = h > maxH ? (h - maxH) / 2 : 0;
  if (w > maxW) w = maxW;
  if (h > maxH) h = maxH;
  n.w = w;
  n.h = h;
  n.base = static_cast<const uint8_t*>(in->base) + cy * in->pitch + cx * n.step * bpp;
  n.ox = ((active - n.k * w) / 2) & ~1;
  n.ty = (kActiveLines - n.v * h) / 2;
  const bool same = cur && cur->pitch == n.pitch && cur->w == n.w && cur->h == n.h
                    && cur->ox == n.ox && cur->ty == n.ty && cur->k == n.k && cur->v == n.v
                    && cur->step == n.step && cur->wide == n.wide;
  if (!(same && cur->base == n.base)) {
    n.gen = same ? cur->gen : ++gen;
    s_src = &n;
  }
  portEXIT_CRITICAL(&s_srcMux);
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
  // PCLK = PLL / div (integer), output clock = module clock.
  REG_WRITE(LCD_CAM_LCD_CLOCK_REG, LCD_CAM_CLK_EN
                                       | (static_cast<uint32_t>(kClocks[s_clk].sel) << LCD_CAM_LCD_CLK_SEL_S)
                                       | (static_cast<uint32_t>(kClocks[s_clk].div) << LCD_CAM_LCD_CLKM_DIV_NUM_S)
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
  memset(blank, 0, s_g.hsync);
  memset(broad, kBlack, kBufBytes);
  memset(broad, 0, s_g.broadLow);
  memset(broad + s_g.half, 0, s_g.broadLow);

  s_line = 0;
  for (int i = 0; i < kBufs; ++i) {
    memcpy(s_buf[i], blank, kBufBytes);
    s_bufType[i] = T_BLANK;
    s_bufGen[i] = 0;
    renderLine(i);
    s_desc[i].dw0 = (1u << 31) | (static_cast<uint32_t>(s_g.len) << 12) | kBufBytes;
    s_desc[i].buf = s_buf[i];
    s_desc[i].next = &s_desc[(i + 1) % kBufs];
  }
  s_produced = kBufs;
  s_dmaLines = 0;
  s_lastDesc = 0;

  s_pins[2] = TVOUT_PIN_HI;
  s_pins[1] = TVOUT_PIN_MID;
  s_pins[0] = isMono(s_std) ? -1 : TVOUT_PIN_LO;

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

// Applies the settings: stops the output, recomputes the geometry and the
// tables, and starts it again when it is on.
void restart() {
  stopHw();
  Range r;
  s_g = makeGeo(isNtsc(s_std), s_clk, s_xoff, s_yoff, &r);
  // keep the stored offsets inside the range of this standard and clock
  if (s_xoff < r.xmin) s_xoff = r.xmin;
  if (s_xoff > r.xmax) s_xoff = r.xmax;
  if (s_yoff < r.ymin) s_yoff = r.ymin;
  if (s_yoff > r.ymax) s_yoff = r.ymax;
  s_white = isMono(s_std) ? 6 : 7;
  buildLut();
  applySource();
  s_mode = TVOUT_OFF;
  if (s_on) {
    if (startHw()) {
      s_mode = s_std;
    } else {
      stopHw();
      s_on = false;
    }
  }
  s_lcdDirty = true;
}

const char kNs[] = "cardputer_emu";

void loadSettings() {
  Preferences prefs;
  prefs.begin(kNs, true);
  const int old = prefs.getUChar("tv_mode", TVOUT_OFF);  // test builds: one value
  s_std = prefs.getUChar("tv_std", old > TVOUT_OFF && old < TVOUT_MODES ? old : TVOUT_PAL_GREY);
  s_on = prefs.getUChar("tv_on", old > TVOUT_OFF ? 1 : 0) != 0;
  s_clk = prefs.getUChar("tv_clk", kClockDef);
  s_xoff = prefs.getChar("tv_x", 0);
  s_yoff = prefs.getChar("tv_y", 0);
  prefs.end();
  if (s_std <= TVOUT_OFF || s_std >= TVOUT_MODES) s_std = TVOUT_PAL_GREY;
  if (s_clk < 0 || s_clk >= kNumClocks) s_clk = kClockDef;
}

void saveSettings() {
  Preferences prefs;
  prefs.begin(kNs, false);
  prefs.putUChar("tv_on", s_on ? 1 : 0);
  prefs.putUChar("tv_std", static_cast<uint8_t>(s_std));
  prefs.putUChar("tv_clk", static_cast<uint8_t>(s_clk));
  prefs.putChar("tv_x", static_cast<int8_t>(s_xoff));
  prefs.putChar("tv_y", static_cast<int8_t>(s_yoff));
  prefs.end();
}

// ---- menu (Fn+T) ----

enum MenuKey { K_NONE = 0, K_UP, K_DOWN, K_LEFT, K_RIGHT, K_MINUS, K_PLUS, K_ENTER, K_BACK, K_T, K_COUNT };
enum { M_OUTPUT = 0, M_STANDARD, M_PICTURE, M_CALIBRATE, M_RESET, M_CLOSE, M_ITEMS };

struct Menu {
  int sel;
  bool calib;
} s_menu;

void setTest(bool on) {
  s_test = on;
}

// One key in the menu. Returns false when the menu should close.
bool menuKey(int k) {
  Menu& m = s_menu;
  if (m.calib) {
    int x = s_xoff, y = s_yoff, c = s_clk;
    switch (k) {
      case K_LEFT: --x; break;
      case K_RIGHT: ++x; break;
      case K_UP: --y; break;
      case K_DOWN: ++y; break;
      case K_MINUS: if (c + 1 < kNumClocks) ++c; break;  // faster clock: narrower
      case K_PLUS: if (c > 0) --c; break;                 // slower clock: wider
      case K_ENTER:
      case K_BACK:
      case K_T:
        m.calib = false;
        setTest(false);
        return true;
      default:
        return true;
    }
    if (x != s_xoff || y != s_yoff || c != s_clk) {
      s_xoff = x;
      s_yoff = y;
      s_clk = c;
      restart();  // clamps the offsets to the new range
    }
    return true;
  }

  switch (k) {
    case K_UP: m.sel = (m.sel + M_ITEMS - 1) % M_ITEMS; return true;
    case K_DOWN: m.sel = (m.sel + 1) % M_ITEMS; return true;
    case K_BACK:
    case K_T:
      return false;
    case K_LEFT:
    case K_RIGHT:
    case K_ENTER:
      break;
    default:
      return true;
  }
  switch (m.sel) {
    case M_OUTPUT:
      s_on = !s_on;
      restart();
      break;
    case M_STANDARD:
      s_std = makeStd(!isNtsc(s_std), isMono(s_std));
      restart();
      break;
    case M_PICTURE:
      s_std = makeStd(isNtsc(s_std), !isMono(s_std));
      restart();
      break;
    case M_CALIBRATE:
      if (k != K_ENTER) break;
      if (!s_on) {  // the picture is needed to calibrate
        s_on = true;
        restart();
      }
      if (s_on) {
        m.calib = true;
        setTest(true);
      }
      break;
    case M_RESET:
      if (k != K_ENTER) break;
      s_clk = kClockDef;
      s_xoff = s_yoff = 0;
      restart();
      break;
    case M_CLOSE:
      if (k == K_ENTER) return false;
      break;
  }
  return true;
}

void menuDraw() {
  auto& lcd = M5Cardputer.Display;
  const uint16_t bg = TFT_BLACK, fg = TFT_WHITE, dim = 0x8410, hl = 0x0010, accent = 0x07FF;
  char buf[48];
  lcd.startWrite();
  lcd.fillScreen(bg);
  lcd.setFont(&fonts::Font2);
  if (s_menu.calib) {
    Range r;
    makeGeo(isNtsc(s_std), s_clk, s_xoff, s_yoff, &r);
    const uint32_t hz = clockHz(s_clk);
    lcd.setTextColor(accent, bg);
    lcd.drawString("TV calibration", 6, 2);
    lcd.setTextColor(fg, bg);
    snprintf(buf, sizeof(buf), "Position  X %+d  Y %+d", s_xoff, s_yoff);
    lcd.drawString(buf, 6, 26);
    snprintf(buf, sizeof(buf), "Width  %d/%d  %lu.%02lu MHz", s_clk + 1, kNumClocks,
             static_cast<unsigned long>(hz / 1000000), static_cast<unsigned long>(hz / 10000 % 100));
    lcd.drawString(buf, 6, 44);
    lcd.setTextColor(dim, bg);
    lcd.drawString("Arrows ; . , /   move", 6, 70);
    lcd.drawString("-  narrower     =  wider", 6, 88);
    lcd.drawString("Enter  back", 6, 106);
  } else {
    static const char* const labels[M_ITEMS] = {"Output", "Standard", "Picture",
                                                "Calibrate picture", "Reset calibration", "Close"};
    lcd.setTextColor(accent, bg);
    lcd.drawString("TV output", 6, 0);
    for (int i = 0; i < M_ITEMS; ++i) {
      const int y = 18 + i * 17;
      const bool sel = i == s_menu.sel;
      const uint16_t rb = sel ? hl : bg;
      if (sel) lcd.fillRect(0, y - 1, lcd.width(), 17, rb);
      lcd.setTextColor(fg, rb);
      lcd.drawString(labels[i], 10, y);
      const char* v = nullptr;
      switch (i) {
        case M_OUTPUT: v = s_on ? "ON" : "OFF"; break;
        case M_STANDARD: v = isNtsc(s_std) ? "NTSC 60 Hz" : "PAL 50 Hz"; break;
        case M_PICTURE: v = isMono(s_std) ? "Black/white" : "6 greys"; break;
        case M_CALIBRATE: v = ">"; break;
      }
      if (v) {
        lcd.setTextColor(i == M_OUTPUT && !s_on ? 0xF800 : accent, rb);
        lcd.drawString(v, 140, y);
      }
    }
    lcd.setFont(&fonts::Font0);
    lcd.setTextColor(dim, bg);
    lcd.drawString(";.: select  ,/ Enter: change  Del: close", 2, 126);
  }
  lcd.setFont(&fonts::Font0);
  lcd.endWrite();
}

// Keys pressed since the last call, with auto-repeat for the arrows.
struct KeyReader {
  uint32_t prev = 0;
  int held = K_NONE;
  uint32_t heldSince = 0, lastRepeat = 0;

  static uint32_t now() {
    M5Cardputer.update();
    const Keyboard_Class::KeysState st = M5Cardputer.Keyboard.keysState();
    uint32_t m = 0;
    for (char c : st.word) {
      switch (c) {
        case ';': m |= 1u << K_UP; break;
        case '.': m |= 1u << K_DOWN; break;
        case ',': m |= 1u << K_LEFT; break;
        case '/': m |= 1u << K_RIGHT; break;
        case '-': case '_': m |= 1u << K_MINUS; break;
        case '=': case '+': m |= 1u << K_PLUS; break;
        case '`': case '~': m |= 1u << K_BACK; break;
        case 't': case 'T': m |= 1u << K_T; break;
      }
    }
    if (st.enter) m |= 1u << K_ENTER;
    if (st.del) m |= 1u << K_BACK;
    if (st.fn || st.space || st.tab) m |= 1u;  // something else held
    return m;
  }

  void init() { prev = now(); }

  int poll(uint32_t ms) {
    const uint32_t cur = now();
    const uint32_t pressed = cur & ~prev & ~1u;
    prev = cur;
    for (int k = 1; k < K_COUNT; ++k) {
      if (pressed & (1u << k)) {
        held = k;
        heldSince = lastRepeat = ms;
        return k;
      }
    }
    const bool repeats = held >= K_UP && held <= K_RIGHT;
    if (repeats && (cur & (1u << held)) && ms - heldSince > 400 && ms - lastRepeat > 90) {
      lastRepeat = ms;
      return held;
    }
    if (!(cur & (1u << held))) held = K_NONE;
    return K_NONE;
  }

  bool anyHeld() { return now() != 0; }
};

}  // namespace

extern "C" {

void tvout_init(void) {
  s_supported = true;
  loadSettings();
  restart();
}

int tvout_mode(void) { return s_mode; }

void tvout_toggle(void) {
  if (!s_supported) return;
  s_on = !s_on;
  restart();
  saveSettings();
}

void tvout_menu(void) {
  if (!s_supported) return;
  s_menuOpen = true;
  delay(60);  // let the LCD task finish the frame it may be drawing
  const int volume = M5Cardputer.Speaker.getVolume();
  M5Cardputer.Speaker.setVolume(0);
  s_menu.sel = M_OUTPUT;
  s_menu.calib = false;
  KeyReader keys;
  keys.init();
  menuDraw();
  for (;;) {
    delay(15);
    const int k = keys.poll(millis());
    if (k == K_NONE) continue;
    if (!menuKey(k)) break;
    menuDraw();
  }
  setTest(false);
  saveSettings();
  // wait for the keys to be released, so that the game does not see them
  for (int i = 0; i < 150 && keys.anyHeld(); ++i) delay(15);
  M5Cardputer.Speaker.setVolume(volume);
  s_lcdDirty = true;
  s_menuOpen = false;
}

void tvout_set_source(const tvout_source_t* in) {
  if (!in || !in->base || in->width <= 0 || in->height <= 0) {
    s_inValid = false;
    s_src = nullptr;
    return;
  }
  if (s_inValid && memcmp(&s_in, in, sizeof(s_in)) == 0) return;  // nothing changed
  s_in = *in;
  s_inValid = true;
  applySource();
}

void tvout_set_luma_rgb(int index, uint8_t r, uint8_t g, uint8_t b) {
  tvout_set_luma(index, static_cast<uint8_t>((77 * r + 150 * g + 29 * b) >> 8));
}

void tvout_set_palette565(const uint16_t* palette, int count, int swapped) {
  for (int i = 0; i < count && i < 256; ++i) {
    uint16_t c = palette[i];
    if (swapped) c = static_cast<uint16_t>(c << 8 | c >> 8);
    tvout_set_luma_rgb(i, ((c >> 11) & 31) * 255 / 31, ((c >> 5) & 63) * 255 / 63, (c & 31) * 255 / 31);
  }
}

void tvout_set_luma(int index, uint8_t luma) {
  s_luma[index & 255] = luma;
  lutEntry(index & 255);
}

int tvout_lcd_wants_frames(void) {
  return !s_menuOpen && (s_mode == TVOUT_OFF || s_lcdDirty);
}

int tvout_lcd_hook(void) {
  if (s_menuOpen) return 1;  // the menu owns the LCD
  if (s_lcdDirty) {
    s_lcdDirty = false;
    auto& lcd = M5Cardputer.Display;
    lcd.fillScreen(TFT_BLACK);
    if (s_mode != TVOUT_OFF) {
      static const char* const names[TVOUT_MODES] = {"", "PAL, 6 greys", "NTSC, 6 greys",
                                                     "PAL, black/white", "NTSC, black/white"};
      lcd.setFont(&fonts::Font2);
      lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      lcd.drawCenterString("TV output on", lcd.width() / 2, lcd.height() / 2 - 26);
      lcd.drawCenterString(names[s_mode], lcd.width() / 2, lcd.height() / 2 - 8);
      lcd.setTextColor(0x8410, TFT_BLACK);
      lcd.drawCenterString("T: off   Fn+T: menu", lcd.width() / 2, lcd.height() / 2 + 14);
      lcd.setFont(&fonts::Font0);
    }
  }
  return s_mode != TVOUT_OFF;
}

}  // extern "C"
