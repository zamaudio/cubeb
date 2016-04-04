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
#include <pthread.h>
#include <sys/time.h>
#include <assert.h>
#include <string.h>
#include <limits.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include "cubeb/cubeb.h"
#include "cubeb-internal.h"

#include "cubeb-speex-resampler.h"

#include <jack/jack.h>
#include <jack/statistics.h>

//static const int MAX_STREAMS = 16;
static const int MAX_CHANNELS  = 8;
static const int FIFO_SIZE = 4096 * sizeof(float);
static const bool AUTO_CONNECT_JACK_PORTS = true;

static void
s16ne_to_float(float *dst, const int16_t *src, size_t n)
{
  for (size_t i = 0; i < n; i++)
    *(dst++) = *(src++) / 32767.0f;
}

typedef enum {
  STATE_INACTIVE,
  STATE_STARTING,
  STATE_STARTED,
  STATE_RUNNING,
  STATE_DRAINING,
  STATE_STOPPING,
  STATE_STOPPED,
  STATE_DRAINED,
} play_state;

static bool
is_running(play_state state)
{
  return state == STATE_STARTING
    || state == STATE_STARTED
    || state == STATE_DRAINING
    || state == STATE_RUNNING;
}

extern "C"
{
/*static*/ int jack_init (cubeb ** context, char const * context_name);
}
static char const * cbjack_get_backend_id(cubeb * context);
static int cbjack_get_max_channel_count(cubeb * ctx, uint32_t * max_channels);
static int cbjack_get_min_latency(cubeb * ctx, cubeb_stream_params params, uint32_t * latency_ms);
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
  .stream_get_latency = NULL,
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

  SpeexResamplerState *resampler;

  play_state state;
  uint64_t position;
  char stream_name[256];
  jack_port_t *output_ports[MAX_CHANNELS];
  float volume;
};

struct cubeb {
  struct cubeb_ops const * ops;
  void *libjack;

  /**< Mutex for stream array */
  pthread_mutex_t mutex;

  /**< Raw input buffer, as filled by data_callback */
  char input_buffer[FIFO_SIZE * MAX_CHANNELS];

  /**< Audio buffer, converted to float */
  float float_interleaved_buffer[FIFO_SIZE * MAX_CHANNELS];

  /**< Audio buffer, at the sampling rate of the output */
  float resampled_interleaved_buffer[FIFO_SIZE * MAX_CHANNELS * 3];

  /**< Non-interleaved audio buffer, used to push to the fifo */
  float buffer[MAX_CHANNELS][FIFO_SIZE];

  cubeb_stream stream;

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
  cubeb_stream *stm = &ctx->stream;

  jack_latency_range_t latency_range;
  jack_nframes_t port_latency, max_latency = 0;

  if (!stm->in_use)
    goto end;
  if (!stm->ports_ready)
    goto end;

  for (i = 0; i < stm->params.channels; ++i) {
      jack_port_get_latency_range(stm->output_ports[i], JackPlaybackLatency, &latency_range);
      port_latency = latency_range.max;
      if (port_latency > max_latency)
          max_latency = port_latency;
  }
  ctx->jack_latency = max_latency;

end:
  return 0;
}

static int
cbjack_process(jack_nframes_t nframes, void *arg)
{
  cubeb *ctx = (cubeb *)arg;
  int t_jack_xruns = ctx->jack_xruns;
  int frames_read = 0;
  int i;

  cubeb_stream *stm = &ctx->stream;
  float to_pre_rate = ( (float)stm->params.rate / (float)stm->context->jack_sample_rate );
  int inputframes = nframes * to_pre_rate;

  int frames_needed = inputframes;
  float *bufs[stm->params.channels];

  // handle xruns by reading and discarding audio that should have been played
  for (i = 0; i < t_jack_xruns; i++) {
      frames_read = cbjack_get_audio_data(stm, ctx->jack_buffer_size * to_pre_rate, true);
  }
  ctx->jack_xruns -= t_jack_xruns;

  if (!stm->in_use)
    goto end;
  if (!stm->ports_ready)
    goto end;

  // get jack output buffers
  for (i = 0; i < stm->params.channels; i++)
      bufs[i] = (float*)jack_port_get_buffer(stm->output_ports[i], nframes);

  frames_read = cbjack_get_audio_data(stm, frames_needed, false);

  if (frames_needed > frames_read) {
      //play silence on buffer underrun
      memset(stm->context->input_buffer + (frames_read * ctx->output_bytes_per_frame), 0,
             (frames_needed - frames_read) * ctx->output_bytes_per_frame);
  }

  cbjack_deinterleave_audio(stm, bufs, inputframes, nframes);

end:
  return 0;
}

static unsigned int
cbjack_get_audio_data(cubeb_stream *stream, unsigned int max_num_frames, bool discard)
{
  unsigned int num_frames = stream->data_callback(stream,
                                                  stream->user_ptr,
                                                  NULL,
                                                  stream->context->input_buffer,
                                                  max_num_frames);
  return discard ? 0 : num_frames;
}

static void
cbjack_deinterleave_audio(cubeb_stream * stream, float **bufs, int inputframes, jack_nframes_t nframes)
{
  float *interleaved_buffer;

  // convert 16-bit to float if needed
  if (stream->params.format == CUBEB_SAMPLE_S16NE) {
    s16ne_to_float(stream->context->float_interleaved_buffer, (short*)stream->context->input_buffer, inputframes * stream->params.channels);
    interleaved_buffer = stream->context->float_interleaved_buffer;
  } else if (stream->params.format == CUBEB_SAMPLE_FLOAT32NE) {
    interleaved_buffer = (float *)stream->context->input_buffer;
  }

  if (stream->resampler != NULL) {
    uint32_t resampler_consumed_frames = inputframes;
    uint32_t resampler_output_frames = (FIFO_SIZE / sizeof(float)) * MAX_CHANNELS * 3;

    int resampler_error = speex_resampler_process_interleaved_float(stream->resampler,
                                                                    interleaved_buffer,
                                                                    &resampler_consumed_frames,
                                                                    stream->context->resampled_interleaved_buffer,
                                                                    &resampler_output_frames);
    assert(resampler_error == 0);
    interleaved_buffer = stream->context->resampled_interleaved_buffer;
    nframes = resampler_output_frames;
  }

  // convert interleaved buffers to contiguous buffers
  for (unsigned int c = 0; c < stream->params.channels; c++) {
    float* buffer = bufs[c];
    for (long f = 0; f < nframes; f++) {
      buffer[f] = interleaved_buffer[(f * stream->params.channels) + c] * stream->volume;
    }
  }
}

static int
load_jack_lib(cubeb* context)
{
  context->libjack = NULL;
  return CUBEB_OK;
}

/*static*/ int
jack_init (cubeb ** context, char const * context_name)
{
  int r;

  if (context == NULL) {
    return CUBEB_ERROR_INVALID_PARAMETER;
  }

  *context = NULL;

  cubeb *ctx = (cubeb*)calloc(1, sizeof(*ctx));
  if (ctx == NULL) {
    return CUBEB_ERROR;
  }

  r = load_jack_lib(ctx);
  if (r != 0) {
    cbjack_destroy(ctx);
    return CUBEB_ERROR;
  }

  r = pthread_mutex_init(&ctx->mutex, NULL);
  if (r != 0) {
    cbjack_destroy(ctx);
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

  ctx->jack_sample_rate = jack_get_sample_rate(ctx->jack_client);
  ctx->jack_xruns = 0;

  jack_set_process_callback (ctx->jack_client, cbjack_process, ctx);
  jack_set_xrun_callback (ctx->jack_client, cbjack_xrun_callback, ctx);
  jack_set_graph_order_callback (ctx->jack_client, cbjack_graph_order_callback, ctx);

  if (jack_activate (ctx->jack_client)) {
    cbjack_destroy(ctx);
    return CUBEB_ERROR;
  }

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
cbjack_get_min_latency(cubeb * ctx, cubeb_stream_params params, uint32_t * latency_ms)
{
  *latency_ms = ctx->jack_latency;
  return CUBEB_OK;
}

static int
cbjack_get_preferred_sample_rate(cubeb * ctx, uint32_t * rate)
{
  *rate = ctx->jack_sample_rate;
  return CUBEB_OK;
}

static void
cbjack_destroy(cubeb * context)
{
  context->active = false;

  if (context->jack_client)
    jack_client_close (context->jack_client);

  pthread_mutex_destroy(&context->mutex);
  free(context);
}

static cubeb_stream*
context_alloc_stream(cubeb * context, char const * stream_name)
{
    if (!context->stream.in_use) {
      cubeb_stream * stm = &context->stream;
      stm->in_use = true;
      snprintf(stm->stream_name, 255, "%s", stream_name);
      return stm;
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

  if (stm->params.rate != stm->context->jack_sample_rate) {
    int resampler_error;
    stm->resampler = speex_resampler_init(stm->params.channels,
                                          stm->params.rate,
                                          stm->context->jack_sample_rate,
                                          10,
                                          &resampler_error);
    if (resampler_error != 0) {
      stm->in_use = false;
      return CUBEB_ERROR;
    }
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

  *stream = stm;

  return CUBEB_OK;
}

static void
cbjack_stream_destroy(cubeb_stream * stream)
{
  stream->state = STATE_INACTIVE;
  stream->ports_ready = false;

  for (unsigned int c = 0; c < stream->params.channels; c++) {
    jack_port_unregister (stream->context->jack_client, stream->output_ports[c]);
    stream->output_ports[c] = NULL;
  }

  if (stream->resampler != NULL) {
    speex_resampler_destroy(stream->resampler);
    stream->resampler = NULL;
  }
  stream->in_use = false;
}

static int
cbjack_stream_start(cubeb_stream * stream)
{
  stream->state = STATE_STARTING;
  return CUBEB_OK;
}

static int
cbjack_stream_stop(cubeb_stream * stream)
{
  stream->state = STATE_STOPPING;
  return CUBEB_OK;
}

static int
cbjack_stream_get_position(cubeb_stream * stream, uint64_t * position)
{
  float const ratio = (float)stream->params.rate / (float)stream->context->jack_sample_rate;
  *position = stream->position * ratio;
  return CUBEB_OK;
}

static int
cbjack_stream_set_volume(cubeb_stream * stm, float volume)
{
  stm->volume = volume;
  return CUBEB_OK;
}
