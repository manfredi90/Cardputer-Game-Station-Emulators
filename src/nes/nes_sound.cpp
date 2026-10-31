#pragma GCC optimize ("Os")

#include <M5Cardputer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

extern "C" {
  #include <osd.h>
  #include <stdbool.h>
}

// Param audio
static constexpr int kSampleRate = 22050; // Hz
static constexpr int kNesHz      = 60;    // 60 FPS
static constexpr int kChunk      = (kSampleRate + kNesHz/2) / kNesHz; // 368 samples per frame

// Canal audio
static constexpr int kChannel    = 0;

// Callback Nofrendo
static void (*s_audio_cb)(void *buffer, int length) = nullptr;

// The emulation thread renders one chunk per frame into a small ring and
// queues it without waiting. A task on core 0 hands the chunks to the
// speaker: playRaw waits there (up to a whole chunk) when the speaker's next
// slot is still busy, which used to stall the emulation for up to 20 ms.
// The speaker plays straight from the buffers, so a buffer is reused only
// once played: when playRaw accepts chunk n, chunks before n - 1 are over.
static constexpr int kBufs = 4;
static int16_t* s_buf[kBufs] = {};
static int16_t* s_scratch = nullptr;       // ring full: render here and drop
static volatile uint32_t s_written = 0;    // chunks queued (emulation thread)
static volatile uint32_t s_done = 0;       // chunks played (audio task)
static QueueHandle_t s_queue = nullptr;
static TaskHandle_t s_task = nullptr;

static void audio_task(void*) {
  uint32_t handed = 0;
  for (;;) {
    uint32_t n;
    if (xQueueReceive(s_queue, &n, portMAX_DELAY) != pdTRUE) continue;
    M5Cardputer.Speaker.playRaw(s_buf[n % kBufs], (size_t)kChunk, (uint32_t)kSampleRate, false /*mono*/,
                                1 /*repeat*/, kChannel, false /*stop_current_sound*/);
    ++handed;
    if (handed >= 2) s_done = handed - 2;
  }
}

extern "C" {

int osd_init_sound() {
  for (int i = 0; i < kBufs; ++i) {
    s_buf[i] = (int16_t*) heap_caps_calloc(kChunk, sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  s_scratch = (int16_t*) heap_caps_calloc(kChunk, sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  auto cfg = M5Cardputer.Speaker.config();
  cfg.sample_rate    = kSampleRate;
  cfg.stereo         = false;   // mono
  cfg.dma_buf_len    = 512;     // buffers DMA
  cfg.dma_buf_count  = 8;
  cfg.task_priority  = 4;       // high priority
  cfg.task_pinned_core = 0;     // pin to core 0 (avoid conflict with main task)
  M5Cardputer.Speaker.config(cfg);

  if (!M5Cardputer.Speaker.isRunning()) {
    M5Cardputer.Speaker.begin();
  }
  M5Cardputer.Speaker.setVolume(80);// 0..255

  M5Cardputer.Speaker.stop(kChannel);

  if (!s_queue) s_queue = xQueueCreate(kBufs, sizeof(uint32_t));
  if (!s_task && s_queue) xTaskCreatePinnedToCore(audio_task, "nesAudio", 3072, nullptr, 5, &s_task, 0);
  return 0;
}

void osd_stopsound() {
  s_audio_cb = nullptr;
  M5Cardputer.Speaker.stop(kChannel);
}

void osd_setsound(void (*playfunc)(void *buffer, int length)) {
  s_audio_cb = playfunc;
}

void osd_getsoundinfo(sndinfo_t *info) {
  info->sample_rate = kSampleRate;
  info->bps         = 16;
}

// Once per frame, from the emulation thread: never waits.
void do_audio_frame() {
  if (!s_audio_cb) return;
  const uint32_t n = s_written;
  if (s_queue && s_buf[n % kBufs] && n - s_done < (uint32_t)kBufs) {
    s_audio_cb((void*)s_buf[n % kBufs], kChunk);
    if (xQueueSend(s_queue, &n, 0) == pdTRUE) s_written = n + 1;
  } else if (s_scratch) {
    s_audio_cb((void*)s_scratch, kChunk);  // keep the APU running, drop the chunk
  }
}

} // extern "C"
