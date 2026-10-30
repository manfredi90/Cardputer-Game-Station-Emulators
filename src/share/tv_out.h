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

// The frame to show: 8-bit pixels addressed through line pointers, width up
// to 256 (drawn with double-width pixels), height up to 240.
void tvout_set_source8(const uint8_t *const *lines, int width, int height);
// Brightness 0..255 of palette entry `index` (0..255).
void tvout_set_luma(int index, uint8_t luma);

// Call from the task that draws the LCD, before drawing a frame. Returns 1
// when the TV is on: the caller should skip the LCD (a notice is shown on it).
int tvout_lcd_hook(void);

#ifdef __cplusplus
}
#endif
