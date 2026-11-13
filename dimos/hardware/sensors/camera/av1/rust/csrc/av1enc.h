// Copyright 2026 Dimensional Inc.
// Licensed under the Apache License, Version 2.0.

// A low-delay SVT-AV1 encoder, narrowed to what a live camera needs.
//
// This shim exists so that nothing in Rust has to mirror EbSvtAv1EncConfiguration or EbBufferHeaderType: both are
// large, and both have changed shape between SVT-AV1 releases (the header even documents a field whose type
// changed). Here the C compiler reads the installed header, so the layout is always the one we are linked
// against, and Rust only sees the five calls below.
#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct dimos_av1_enc dimos_av1_enc;

// Opens an encoder for `width`x`height` 8-bit I420 at `fps`, CBR at `bitrate_bps`, a keyframe every `gop`
// frames, SVT-AV1 speed `preset` (0 slowest .. 13 fastest) on `threads` cores, predicting only from the
// previous frame (low delay). Returns NULL with `error` filled on failure.
dimos_av1_enc* dimos_av1_open(uint32_t width, uint32_t height, uint32_t fps, uint32_t bitrate_bps, uint32_t gop,
                              int preset, uint32_t threads, char* error, size_t error_len);

// The sequence header for this stream, valid until close. A decoder that joins mid-stream needs it, so the
// caller puts it in front of every keyframe (what `h264parse config-interval=-1` does for H.264).
void dimos_av1_stream_header(const dimos_av1_enc* enc, const uint8_t** data, size_t* len);

// Feeds one I420 frame. `pts` counts frames. 0 on success, -1 with `error` filled otherwise.
int dimos_av1_send(dimos_av1_enc* enc, const uint8_t* y, const uint8_t* u, const uint8_t* v, uint32_t y_stride,
                   uint32_t uv_stride, int64_t pts, char* error, size_t error_len);

// The next encoded temporal unit, which in low delay is the one for the frame just sent, so this blocks until
// it is ready (SVT-AV1 documents get_packet as blocking under pred-struct=1). 1: a packet, borrowed until
// `dimos_av1_release`. 0: the encoder is done and has no more. -1: error, with `error` filled.
int dimos_av1_next(dimos_av1_enc* enc, const uint8_t** data, size_t* len, int* keyframe, int64_t* pts,
                   char* error, size_t error_len);

// Hands the packet from the last `dimos_av1_next` back to the encoder's pool.
void dimos_av1_release(dimos_av1_enc* enc);

void dimos_av1_close(dimos_av1_enc* enc);
