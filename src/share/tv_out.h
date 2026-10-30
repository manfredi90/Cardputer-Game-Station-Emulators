#pragma once
// Composite video (CVBS) output to a TV, Cardputer ADV only.
//
// 240p PAL (50 Hz) or NTSC (60 Hz), 6 grey levels with 3 GPIOs or black and
// white with 2 GPIOs. The ESP32-S3 has no DAC: the levels come from GPIOs and
// resistors, the LCD_CAM peripheral streams the samples with DMA, and an IRAM
// interrupt renders each line from the emulator's own frame buffer just
// before it is sent, so no extra frame buffer is needed.
//
//   G5 --[ 270 R ]--+
//   G6 --[ 560 R ]--+----> RCA centre (video)     GND ----> RCA shield
//   G3 --[ 1.1 k ]--+      (G3 only for 6 greys; mono also works with 220 R on G5)
//
// The T key cycles OFF -> PAL grey -> NTSC grey -> PAL mono -> NTSC mono.
// The mode is saved and restored the next time a core with TV support starts.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  TVOUT_OFF = 0,
  TVOUT_PAL_GREY,
  TVOUT_NTSC_GREY,
  TVOUT_PAL_MONO,
  TVOUT_NTSC_MONO,
  TVOUT_MODES
};

// Enables TV support for the running core and restores the saved mode.
void tvout_init(void);
int tvout_mode(void);
void tvout_cycle_mode(void);

enum {
  TVOUT_FMT_INDEX8 = 0,       // 8-bit palette index (brightness from tvout_set_luma*)
  TVOUT_FMT_RGB565,           // 16-bit RGB565, native byte order
  TVOUT_FMT_RGB565_SWAPPED,   // 16-bit RGB565, bytes exchanged (big endian)
  TVOUT_FMT_BGR444            // 16-bit 0000BBBB GGGGRRRR
};

// The frame to show, read directly by the interrupt: it must stay valid (in
// internal RAM) until the next call. Call again whenever the buffer or the
// geometry changes (cheap when nothing changed). Frames that do not fit are
// cropped around the centre; smaller ones are centred.
typedef struct {
  const void *base;   // first pixel
  int pitch;          // bytes per line
  int width, height;  // pixels
  uint8_t format;     // TVOUT_FMT_*
  uint8_t xscale;     // samples per pixel shown: 1..3 (100 ns each, 512 per line)
  uint8_t yscale;     // TV lines per source line: 1 or 2 (240 lines)
  uint8_t xstep;      // 2 = show every other pixel (wide modes), else 1
} tvout_source_t;

void tvout_set_source(const tvout_source_t *source);

// Brightness 0..255 of palette entry `index` (0..255), for TVOUT_FMT_INDEX8.
void tvout_set_luma(int index, uint8_t luma);
void tvout_set_luma_rgb(int index, uint8_t r, uint8_t g, uint8_t b);
void tvout_set_palette565(const uint16_t *palette, int count, int swapped);

// Call from the task that draws the LCD, before drawing a frame. Returns 1
// when the TV is on: the caller should skip the LCD (a notice is shown on it).
int tvout_lcd_hook(void);
// For cores that copy or queue frames for the LCD task: 0 while the TV is on
// and the LCD has nothing new to show, so that work can be skipped. The
// source must still be passed to tvout_set_source.
int tvout_lcd_wants_frames(void);

#ifdef __cplusplus
}
#endif
