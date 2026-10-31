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
// T switches the output on and off. Fn+T opens a menu on the LCD: standard
// (PAL / NTSC), picture (6 greys / black and white) and calibration
// (position, and width through five sample clocks with an integer divider).
// The settings are saved and restored when a core with TV support starts.

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

// Enables TV support for the running core and restores the saved settings.
void tvout_init(void);
// TVOUT_OFF, or the running standard.
int tvout_mode(void);
// T key: output on / off (does nothing in cores without TV support).
void tvout_toggle(void);
// Fn+T: settings menu on the LCD; returns when it is closed. The calling
// task (the core's input polling) is blocked meanwhile, which pauses most
// cores; the speaker is muted.
void tvout_menu(void);

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
// Frame pacing locked to the TV, against tearing. Call once per frame from the
// emulation loop, where the core would wait for its next frame, with the
// core's frame rate. While the TV is on and the rate is close to the
// standard's, the TV field rate is tuned to it and the call waits until the
// TV has shown the last line of the picture: the next frame is then written
// ahead of the beam. Returns 0 when the core paces itself as usual, else the
// core must skip its own wait (and restart its timing from now): 1 = on time,
// 2 = the frame was late (the call did not wait); the next frame should then
// be emulated without drawing it, if the core can, since it starts while the
// TV is showing the picture.
int tvout_frame_sync(float fps);

// For cores that copy or queue frames for the LCD task: 0 while the TV is on
// and the LCD has nothing new to show, so that work can be skipped. The
// source must still be passed to tvout_set_source.
int tvout_lcd_wants_frames(void);

#ifdef __cplusplus
}
#endif
