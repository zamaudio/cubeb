/*
 * Copyright © 2012 David Richards
 * Copyright © 2013 Sebastien Alaiwan
 * Copyright © 2016 Damien Zammit
 *
 * This program is made available under an ISC-style license.  See the
 * accompanying file LICENSE for details.
 */
#define _DEFAULT_SOURCE
#define _BSD_SOURCE
#define _POSIX_SOURCE
#include <algorithm>
#include <dlfcn.h>
#include <limits>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <string.h>
#include <limits.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include "cubeb/cubeb.h"
#include "cubeb-internal.h"
#include "cubeb_resampler.h"

#include <jack/jack.h>
#include <jack/statistics.h>

static const int MAX_STREAMS = 16;
static const int MAX_CHANNELS  = 8;
static const int FIFO_SIZE = 4096 * sizeof(float);
static const bool AUTO_CONNECT_JACK_PORTS = true;

static void
s16ne_to_float(float *dst, const int16_t *src, size_t n)
{
  for (size_t i = 0; i < n; i++)
    *(dst++) = (float)((float)*(src++) / 32767.0f);
}

extern "C"
{
/*static*/ int jack_init (cubeb ** context, char const * context_name);
}
static char const * cbjack_get_backend_id(cubeb * context);
static int cbjack_get_max_channel_count(cubeb * ctx, uint32_t * max_channels);
static int cbjack_get_min_latency(cubeb * ctx, cubeb_stream_params params, uint32_t * latency_ms);
static int cbjack_get_latency(cubeb_stream * stm, unsigned int * latency_ms);
static int cbjack_get_preferred_sample_rate(cubeb * ctx, uint32_t * rate);
static void cbjack_destroy(cubeb * context);
static void cbjack_deinterleave_audio(cubeb_stream * stream, float **bufs, int inputframes, jack_nframes_t nframes);
static unsigned int cbjack_get_audio_data(cubeb_stream *stream, unsigned int bytes, bool discard);
static int cbjack_stream_init(cubeb * context, cubeb_stream ** stream, char const * stream_name,
                              cubeb_devid input_device,
                              cubeb_stream_params * input_stream_params,
                              cubeb_devid output_device,
                              cubeb_stream_params * output_stream_params,
                              unsigned int latency,
                              cubeb_data_callback data_callback,
                              cubeb_state_callback state_callback,
                              void * user_ptr);
static void cbjack_stream_destroy(cubeb_stream * stream);
static int cbjack_stream_start(cubeb_stream * stream);
static int cbjack_stream_stop(cubeb_stream * stream);
static int cbjack_stream_get_position(cubeb_stream * stream, uint64_t * position);
static int cbjack_stream_set_volume(cubeb_stream * stm, float volume);

static struct cubeb_ops const cbjack_ops = {
  .init = jack_init,
  .get_backend_id = cbjack_get_backend_id,
  .get_max_channel_count = cbjack_get_max_channel_count,
  .get_min_latency = cbjack_get_min_latency,
  .get_preferred_sample_rate = cbjack_get_preferred_sample_rate,
  .enumerate_devices = NULL,
  .destroy = cbjack_destroy,
  .stream_init = cbjack_stream_init,
  .stream_destroy = cbjack_stream_destroy,
  .stream_start = cbjack_stream_start,
  .stream_stop = cbjack_stream_stop,
  .stream_get_position = cbjack_stream_get_position,
  .stream_get_latency = cbjack_get_latency,
  .stream_set_volume = cbjack_stream_set_volume,
  .stream_set_panning = NULL,
  .stream_get_current_device = NULL,
  .stream_device_destroy = NULL,
  .stream_register_device_changed_callback = NULL,
  .register_device_collection_changed = NULL
};

struct cubeb_stream {
  cubeb *context;

  bool in_use; /**< Set to false iff the stream is free */
  bool ports_ready; /**< Set to true iff the JACK ports are ready */

  cubeb_data_callback data_callback;
  cubeb_state_callback state_callback;
  void *user_ptr;
  cubeb_stream_params params;
  cubeb_stream_params out_params;

  cubeb_resampler *resampler;

  uint64_t position;
  bool pause;
  char stream_name[256];
  jack_port_t *output_ports[MAX_CHANNELS];
  float volume;
};

struct cubeb {
  struct cubeb_ops const * ops;
  void *libjack;

  /**< Raw input buffer, as filled by data_callback */
  char input_buffer[FIFO_SIZE * MAX_CHANNELS];

  /**< Audio buffer, converted to float */
  float float_interleaved_buffer[FIFO_SIZE * MAX_CHANNELS];

  /**< Audio buffer, at the sampling rate of the output */
  float resampled_interleaved_buffer_float[FIFO_SIZE * MAX_CHANNELS * 3];
  int16_t resampled_interleaved_buffer_s16ne[FIFO_SIZE * MAX_CHANNELS * 3];

  /**< Non-interleaved audio buffer, used to push to the fifo */
  float buffer[MAX_CHANNELS][FIFO_SIZE];

  cubeb_stream streams[MAX_STREAMS];
  unsigned int active_streams;

  bool active;
  unsigned int jack_sample_rate;
  unsigned int jack_latency;
  unsigned int jack_xruns;
  unsigned int jack_buffer_size;
  unsigned int fragment_size;
  unsigned int output_bytes_per_frame;
  jack_client_t *jack_client;
};

static void
cbjack_connect_ports (cubeb_stream * stream)
{
  const char **physical_ports = jack_get_ports (stream->context->jack_client,
                                                    NULL, NULL, JackPortIsInput | JackPortIsPhysical);
  if (physical_ports == NULL) {
    return;
  }

  // Connect to all physical ports
  for (unsigned int c = 0; c < stream->params.channels && physical_ports[c]; c++) {
    const char *src_port = jack_port_name (stream->output_ports[c]);

    jack_connect (stream->context->jack_client, src_port, physical_ports[c]);
  }
  jack_free(physical_ports);
}

static int
cbjack_xrun_callback(void *arg)
{
  cubeb *ctx = (cubeb *)arg;

  float delay = jack_get_xrun_delayed_usecs(ctx->jack_client);
  int fragments = (int)ceilf( ((delay / 1000000.0) * ctx->jack_sample_rate )
                             / (float)(ctx->jack_buffer_size) );
  ctx->jack_xruns += fragments;
  return 0;
}

static int
cbjack_graph_order_callback(void *arg)
{
  cubeb *ctx = (cubeb *)arg;
  int i;

  jack_latency_range_t latency_range;
  jack_nframes_t port_latency, max_latency = 0;

  for (int j = 0; j < MAX_STREAMS; j++) {
    cubeb_stream *stm = &ctx->streams[j];

    if (!stm->in_use)
      continue;
    if (!stm->ports_ready)
      continue;

    for (i = 0; i < (int)stm->params.channels; ++i) {
      jack_port_get_latency_range(stm->output_ports[i], JackPlaybackLatency, &latency_range);
      port_latency = latency_range.max;
      if (port_latency > max_latency)
          max_latency = port_latency;
    }
    ctx->jack_latency = max_latency;
  }
  return 0;
}

static int
cbjack_process(jack_nframes_t nframes, void *arg)
{
  cubeb *ctx = (cubeb *)arg;
  int t_jack_xruns = ctx->jack_xruns;
  int frames_read = 0;
  int i;
  int to_pre_rate;
  int inputframes;
  int frames_needed;

  for (int j = 0; j < MAX_STREAMS; j++) {
    cubeb_stream *stm = &ctx->streams[j];
    float *bufs[stm->params.channels];

    if (!stm->in_use)
      continue;

    to_pre_rate = 1.f / ( (float)stm->params.rate / (float)ctx->jack_sample_rate );
    inputframes = nframes * to_pre_rate;

    frames_needed = inputframes;

    // handle xruns by reading and discarding audio that should have been played
    for (i = 0; i < t_jack_xruns; i++) {
        frames_read = cbjack_get_audio_data(stm, ctx->jack_buffer_size * to_pre_rate, true);
        stm->position += frames_read * ctx->output_bytes_per_frame;
    }
    ctx->jack_xruns -= t_jack_xruns;


    if (!stm->ports_ready)
      continue;

    // get jack output buffers
    for (i = 0; i < (int)stm->params.channels; i++)
      bufs[i] = (float*)jack_port_get_buffer(stm->output_ports[i], nframes);

    if (stm->pause) {
      // paused, play silence
      for (unsigned int c = 0; c < stm->params.channels; c++) {
        float* buffer = bufs[c];
        for (long f = 0; f < nframes; f++) {
          buffer[f] = 0.f;
        }
      }
    } else {
      // unpaused, play audio
      cbjack_deinterleave_audio(stm, bufs, frames_needed, nframes);
    }
  }

  return 0;
}

static unsigned int
cbjack_get_audio_data(cubeb_stream *stream, unsigned int max_num_frames, bool discard)
{
  unsigned int num_frames = stream->data_callback(stream,
                                                  stream->user_ptr,
                                                  NULL,
                                                  stream->context->input_buffer,
                                                  max_num_frames * stream->context->output_bytes_per_frame);
  return num_frames;
}

static void
cbjack_deinterleave_audio(cubeb_stream * stream, float **bufs, int inputframes, jack_nframes_t nframes)
{
  float *interleaved_buffer;

  float *input_float = (float *)stream->context->input_buffer;
  int16_t *input_s16ne = (int16_t *)stream->context->input_buffer;

    long resampler_input_frames = inputframes;
    long resampler_output_frames = nframes;
    long resampler_outframes = 0;

  if (stream->params.format == CUBEB_SAMPLE_S16NE) {
    resampler_outframes = cubeb_resampler_fill(stream->resampler,
                                                   input_s16ne,
                                                   &resampler_input_frames,
                                                   stream->context->resampled_interleaved_buffer_s16ne,
                                                   resampler_output_frames);

    s16ne_to_float(stream->context->resampled_interleaved_buffer_float, stream->context->resampled_interleaved_buffer_s16ne, resampler_output_frames * stream->params.channels);
  } else if (stream->params.format == CUBEB_SAMPLE_FLOAT32NE) {
    resampler_outframes = cubeb_resampler_fill(stream->resampler,
                                                   input_float,
                                                   &resampler_input_frames,
                                                   stream->context->resampled_interleaved_buffer_float,
                                                   resampler_output_frames);
  }
  interleaved_buffer = stream->context->resampled_interleaved_buffer_float;

  // convert interleaved buffers to contiguous buffers
  for (unsigned int c = 0; c < stream->params.channels; c++) {
    float* buffer = bufs[c];
    for (long f = 0; f < resampler_outframes; f++) {
      buffer[f] = interleaved_buffer[(f * stream->params.channels) + c] * stream->volume;
    }
    for (long f = resampler_outframes; f < nframes; f++) {
      buffer[f] = 0.f;
    }
  }
  // advance stream position
  stream->position += resampler_outframes * stream->context->output_bytes_per_frame;
}

/*static*/ int
jack_init (cubeb ** context, char const * context_name)
{
  if (context == NULL) {
    return CUBEB_ERROR_INVALID_PARAMETER;
  }

  *context = NULL;

  cubeb *ctx = (cubeb*)calloc(1, sizeof(*ctx));
  if (ctx == NULL) {
    return CUBEB_ERROR;
  }

  ctx->ops = &cbjack_ops;

  const char* jack_client_name = "cubeb";
  if (context_name)
    jack_client_name = context_name;

  ctx->jack_client = jack_client_open(jack_client_name,
                                          JackNoStartServer,
                                          NULL);

  if (ctx->jack_client == NULL) {
    cbjack_destroy(ctx);
    return CUBEB_ERROR;
  }

  ctx->jack_xruns = 0;

  jack_set_process_callback (ctx->jack_client, cbjack_process, ctx);
  jack_set_xrun_callback (ctx->jack_client, cbjack_xrun_callback, ctx);
  jack_set_graph_order_callback (ctx->jack_client, cbjack_graph_order_callback, ctx);

  if (jack_activate (ctx->jack_client)) {
    cbjack_destroy(ctx);
    return CUBEB_ERROR;
  }

  ctx->jack_sample_rate = jack_get_sample_rate(ctx->jack_client);

  ctx->active = true;
  *context = ctx;

  return CUBEB_OK;
}

static char const *
cbjack_get_backend_id(cubeb * context)
{
  return "jack";
}

static int
cbjack_get_max_channel_count(cubeb * ctx, uint32_t * max_channels)
{
  *max_channels = MAX_CHANNELS;
  return CUBEB_OK;
}

static int
cbjack_get_latency(cubeb_stream * stm, unsigned int * latency_ms)
{
  *latency_ms = stm->context->jack_latency;
  return CUBEB_OK;
}

static int
cbjack_get_min_latency(cubeb * ctx, cubeb_stream_params params, uint32_t * latency_ms)
{
  *latency_ms = ctx->jack_latency;
  return CUBEB_OK;
}

static int
cbjack_get_preferred_sample_rate(cubeb * ctx, uint32_t * rate)
{
  *rate = jack_get_sample_rate(ctx->jack_client);
  return CUBEB_OK;
}

static void
cbjack_destroy(cubeb * context)
{
  context->active = false;
  if (context->jack_client != NULL)
    jack_client_close (context->jack_client);
  free(context);
}

static cubeb_stream*
context_alloc_stream(cubeb * context, char const * stream_name)
{
  for (int i = 0; i < MAX_STREAMS; i++) {
    if (!context->streams[i].in_use) {
      cubeb_stream * stm = &context->streams[i];
      stm->in_use = true;
      snprintf(stm->stream_name, 255, "%s_%u", stream_name, i);
      return stm;
    }
  }
  return NULL;
}

static int
cbjack_stream_init(cubeb * context, cubeb_stream ** stream, char const * stream_name,
                   cubeb_devid input_device,
                   cubeb_stream_params * input_stream_params,
                   cubeb_devid output_device,
                   cubeb_stream_params * output_stream_params,
                   unsigned int latency,
                   cubeb_data_callback data_callback,
                   cubeb_state_callback state_callback,
                   void * user_ptr)
{
  assert(!input_stream_params && "not supported.");
  if (input_device || output_device) {
    /* Device selection not yet implemented. */
    return CUBEB_ERROR_DEVICE_UNAVAILABLE;
  }

  if (stream == NULL || output_stream_params == NULL) {
    return CUBEB_ERROR_INVALID_PARAMETER;
  }

  if (output_stream_params->format != CUBEB_SAMPLE_FLOAT32NE &&
      output_stream_params->format != CUBEB_SAMPLE_S16NE) {
    return CUBEB_ERROR_INVALID_FORMAT;
  }

  *stream = NULL;

  // Find a free stream.
  cubeb_stream * stm = context_alloc_stream(context, stream_name);

  // No free stream?
  if (stm == NULL) {
    return CUBEB_ERROR;
  }

  stm->ports_ready = false;
  stm->user_ptr = user_ptr;
  stm->context = context;
  stm->params = *output_stream_params;
  stm->data_callback = data_callback;
  stm->state_callback = state_callback;
  stm->position = 0;
  stm->volume = 1.0f;
  if (stm->params.format == CUBEB_SAMPLE_FLOAT32NE) {
      context->output_bytes_per_frame = sizeof(float);
  } else {
      context->output_bytes_per_frame = sizeof(short);
  }
  context->jack_buffer_size = jack_get_buffer_size(context->jack_client);
  context->fragment_size = context->jack_buffer_size * context->output_bytes_per_frame;

  stm->resampler = NULL;

    stm->out_params.format = stm->params.format;
    stm->out_params.rate = jack_get_sample_rate(context->jack_client);
    stm->out_params.channels = stm->params.channels;
    stm->resampler = cubeb_resampler_create(stm,
                                            &stm->out_params,
                                            &stm->out_params,
                                            stm->params.rate,
                                            stm->data_callback,
                                            stm->user_ptr,
                                            CUBEB_RESAMPLER_QUALITY_DESKTOP);
    if (!stm->resampler) {
      stm->in_use = false;
      return CUBEB_ERROR;
    }

  for (unsigned int c = 0; c < stm->params.channels; c++) {
    char portname[256];
    snprintf(portname, 255, "%s_%d", stm->stream_name, c);
    stm->output_ports[c] = jack_port_register(stm->context->jack_client,
                                                  portname,
                                                  JACK_DEFAULT_AUDIO_TYPE,
                                                  JackPortIsOutput,
                                                  0);
  }

  if (AUTO_CONNECT_JACK_PORTS) {
    cbjack_connect_ports(stm);
  }

  stm->ports_ready = true;
  stm->pause = true;

  *stream = stm;

  return CUBEB_OK;
}

static void
cbjack_stream_destroy(cubeb_stream * stream)
{
  stream->ports_ready = false;

  for (unsigned int c = 0; c < stream->params.channels; c++) {
    if (stream->output_ports[c]) {
      jack_port_unregister (stream->context->jack_client, stream->output_ports[c]);
      stream->output_ports[c] = NULL;
    }
  }

  if (stream->resampler) {
    cubeb_resampler_destroy(stream->resampler);
    stream->resampler = NULL;
  }
  stream->in_use = false;
}

static int
cbjack_stream_start(cubeb_stream * stream)
{
  stream->volume = 1.f;
  stream->pause = false;
  return CUBEB_OK;
}

static int
cbjack_stream_stop(cubeb_stream * stream)
{
  stream->volume = 0.f;
  stream->pause = true;
  return CUBEB_OK;
}

static int
cbjack_stream_get_position(cubeb_stream * stream, uint64_t * position)
{
  *position = stream->position;
  return CUBEB_OK;
}

static int
cbjack_stream_set_volume(cubeb_stream * stm, float volume)
{
  stm->volume = volume;
  return CUBEB_OK;
}
