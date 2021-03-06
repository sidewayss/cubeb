/*
* Copyright © 2016 Mozilla Foundation
*
* This program is made available under an ISC-style license.  See the
* accompanying file LICENSE for details.
*/

/* libcubeb api/function test. Loops input back to output and check audio
* is flowing. */
#include "gtest/gtest.h"
#if !defined(_XOPEN_SOURCE)
#define _XOPEN_SOURCE 600
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory>
#include "cubeb/cubeb.h"
#include "cubeb_log.h"
#include "common.h"
#include <atomic>

#define SAMPLE_FREQUENCY 48000
#define STREAM_FORMAT CUBEB_SAMPLE_FLOAT32LE

struct user_state_duplex
{
  std::atomic<int> seen_audio{ 0 };
  std::atomic<long> position;
  LONGLONG start_time;
  LONGLONG frequency;
  LONGLONG tone_start;
  bool is_tone = false;
  bool is_max = false;
};

long data_cb_duplex(cubeb_stream * stream, void * user, const void * inputbuffer, void * outputbuffer, long nframes)
{
  user_state_duplex * u = reinterpret_cast<user_state_duplex*>(user);
  float *ib = (float *)inputbuffer;
  float *ob = (float *)outputbuffer;
  bool seen_audio = 1;
  float max = 0.0;
  float t1, t2;

  if (stream == NULL || inputbuffer == NULL || outputbuffer == NULL) {
    return CUBEB_ERROR;
  }

  // Loop back: upmix the single input channel to the two output channels,
  // checking if there is noise in the process.
  LARGE_INTEGER now;
  LONGLONG current_time;
  double intpart;
  double seconds;
  double remainder;
  QueryPerformanceCounter(&now);
  current_time = now.QuadPart;
  long output_index = 0;
  seconds = 1.0 * (current_time - u->start_time) / u->frequency;
  remainder = modf(seconds, &intpart);
  for (long i = 0; i < nframes; i++) {
    if (ib[i] <= -1.0 || ib[i] >= 1.0) {
      seen_audio = 0;
      break;
    }

    if (intpart >= 0) {
      if (abs(ib[i]) > max)
        max = abs(ib[i]);
      if (max > 0.1 && !u->is_max) {
        QueryPerformanceCounter(&now);
        ALOGV("elapsed:%.2f", 1000.0 * (now.QuadPart - u->tone_start) / u->frequency);
        u->is_max = true;
      }
      if (remainder > 0.150 && remainder < 0.160) {
        if (!u->is_tone) {
          u->is_tone = true;
          u->is_max  = false;
          u->tone_start = current_time;
        }
        t1 = sin(2 * M_PI*(i + u->position) * 350 / SAMPLE_FREQUENCY);
        t2 = sin(2 * M_PI*(i + u->position) * 440 / SAMPLE_FREQUENCY);
        ob[output_index] /*= ob[output_index + 1]*/ = (((SHRT_MAX / 2) * t1) + ((SHRT_MAX / 2) * t2)) / 32767;
        output_index++; // += 2;
      }
      else {
        if (u->is_tone) {
          u->is_tone = false;
        }
        ob[output_index] /*= ob[output_index + 1]*/ = 0; // ib[i];
        output_index++; // += 2;
      }
    }
  }

  u->position += nframes;
  u->seen_audio |= seen_audio;

  return nframes;
}

void state_cb_duplex(cubeb_stream * stream, void * /*user*/, cubeb_state state)
{
  if (stream == NULL)
    return;

  switch (state) {
  case CUBEB_STATE_STARTED:
    fprintf(stderr, "stream started\n"); break;
  case CUBEB_STATE_STOPPED:
    fprintf(stderr, "stream stopped\n"); break;
  case CUBEB_STATE_DRAINED:
    fprintf(stderr, "stream drained\n"); break;
  default:
    fprintf(stderr, "unknown stream state %d\n", state);
  }

  return;
}

TEST(cubeb, duplex)
{
  cubeb *ctx;
  cubeb_stream *stream;
  cubeb_device_type dev_type;
  cubeb_stream_params inp_params;
  cubeb_stream_params out_params;
  uint32_t inp_frames = 0;
  uint32_t out_frames = 0;
  user_state_duplex stream_state;
  int r;

  r = common_init(&ctx, "Cubeb duplex example");
  ASSERT_EQ(r, CUBEB_OK) << "Error initializing cubeb library";

  std::unique_ptr<cubeb, decltype(&cubeb_destroy)>
    cleanup_cubeb_at_exit(ctx, cubeb_destroy);

  /* This test needs an available input device, skip it if this host does not
   * have one. */
  if (!has_available_input_device(ctx)) {
    return;
  }

  /* minimal latency user-case: input and output params identical. */
  inp_params.format     = STREAM_FORMAT;
  out_params.format     = STREAM_FORMAT;
  inp_params.layout     = CUBEB_LAYOUT_MONO;
  out_params.layout     = CUBEB_LAYOUT_MONO;
  inp_params.channels   = 1;
  out_params.channels   = 1;
  inp_params.rate       = 44100;
  out_params.rate       = 44100;

  dev_type = (cubeb_device_type)(CUBEB_DEVICE_TYPE_INPUT | CUBEB_DEVICE_TYPE_OUTPUT);
  r = cubeb_get_min_latency(ctx, dev_type, NULL, NULL, &inp_params, &out_params);
  ASSERT_EQ(r, CUBEB_OK) << "Could not get minimum latency";

  r = cubeb_stream_init(ctx, &stream, "Cubeb duplex",
                        NULL, &inp_params, NULL, &out_params, 1, // 1 because latency cannot be <1 in cubeb.c
                        data_cb_duplex, state_cb_duplex, &stream_state);
  ASSERT_EQ(r, CUBEB_OK) << "Error initializing cubeb stream";

  std::unique_ptr<cubeb_stream, decltype(&cubeb_stream_destroy)>
    cleanup_stream_at_exit(stream, cubeb_stream_destroy);
    
  LARGE_INTEGER now;
  QueryPerformanceFrequency(&now);
  stream_state.frequency = now.QuadPart;
  QueryPerformanceCounter(&now);
  stream_state.start_time = now.QuadPart;

  cubeb_stream_start(stream);
  delay(60600);
  cubeb_stream_stop(stream);

  ASSERT_TRUE(stream_state.seen_audio.load());
}
 