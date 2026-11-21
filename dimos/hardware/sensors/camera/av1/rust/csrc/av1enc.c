// Copyright 2026 Dimensional Inc.
// Licensed under the Apache License, Version 2.0.

#include "av1enc.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <svt-av1/EbSvtAv1Enc.h>

struct dimos_av1_enc {
    EbComponentType* handle;
    uint32_t width;
    uint32_t height;
    // Reused for every frame: SVT-AV1 reads the picture through p_buffer and copies what it needs.
    EbBufferHeaderType input;
    EbSvtIOFormat picture;
    // The sequence header, kept so a keyframe can carry it.
    EbBufferHeaderType* header;
    // The packet handed out by the last dimos_av1_next, still owned by the encoder's pool.
    EbBufferHeaderType* out;
};

static void say(char* error, size_t error_len, const char* format, ...) {
    if (error == NULL || error_len == 0) {
        return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf(error, error_len, format, args);
    va_end(args);
}

dimos_av1_enc* dimos_av1_open(uint32_t width, uint32_t height, uint32_t fps, uint32_t bitrate_bps, uint32_t gop,
                              int preset, uint32_t threads, char* error, size_t error_len) {
    dimos_av1_enc* enc = calloc(1, sizeof(*enc));
    if (enc == NULL) {
        say(error, error_len, "out of memory");
        return NULL;
    }
    EbSvtAv1EncConfiguration config;
    memset(&config, 0, sizeof(config));
    EbErrorType status = svt_av1_enc_init_handle(&enc->handle, &config);
    if (status != EB_ErrorNone) {
        say(error, error_len, "svt_av1_enc_init_handle: 0x%x", status);
        free(enc);
        return NULL;
    }
    config.source_width = width;
    config.source_height = height;
    config.frame_rate_numerator = fps;
    config.frame_rate_denominator = 1;
    config.encoder_bit_depth = 8;
    // CBR, because the link this goes over has a budget and a VBR burst is what shows up as a freeze.
    config.rate_control_mode = 2;
    config.target_bit_rate = bitrate_bps;
    // Each frame predicts only from the one before it. Without this SVT-AV1 looks ahead, which costs about
    // 100 ms before the first frame can leave.
    config.pred_structure = LOW_DELAY;
    // intra_period_length counts the frames BETWEEN keyframes, so a gop of n frames is n - 1.
    config.intra_period_length = (int32_t)gop - 1;
    config.enc_mode = (int8_t)preset;
    config.level_of_parallelism = threads;
    status = svt_av1_enc_set_parameter(enc->handle, &config);
    if (status != EB_ErrorNone) {
        say(error, error_len, "svt_av1_enc_set_parameter: 0x%x", status);
        svt_av1_enc_deinit_handle(enc->handle);
        free(enc);
        return NULL;
    }
    status = svt_av1_enc_init(enc->handle);
    if (status != EB_ErrorNone) {
        say(error, error_len, "svt_av1_enc_init: 0x%x", status);
        svt_av1_enc_deinit_handle(enc->handle);
        free(enc);
        return NULL;
    }
    status = svt_av1_enc_stream_header(enc->handle, &enc->header);
    if (status != EB_ErrorNone || enc->header == NULL) {
        say(error, error_len, "svt_av1_enc_stream_header: 0x%x", status);
        svt_av1_enc_deinit(enc->handle);
        svt_av1_enc_deinit_handle(enc->handle);
        free(enc);
        return NULL;
    }
    enc->width = width;
    enc->height = height;
    enc->input.size = sizeof(EbBufferHeaderType);
    enc->input.p_buffer = (uint8_t*)&enc->picture;
    return enc;
}

void dimos_av1_stream_header(const dimos_av1_enc* enc, const uint8_t** data, size_t* len) {
    *data = enc->header->p_buffer;
    *len = enc->header->n_filled_len;
}

int dimos_av1_send(dimos_av1_enc* enc, const uint8_t* y, const uint8_t* u, const uint8_t* v, uint32_t y_stride,
                   uint32_t uv_stride, int64_t pts, char* error, size_t error_len) {
    // SVT-AV1's input is non-const but it only reads the planes.
    enc->picture.luma = (uint8_t*)y;
    enc->picture.cb = (uint8_t*)u;
    enc->picture.cr = (uint8_t*)v;
    enc->picture.y_stride = y_stride;
    enc->picture.cb_stride = uv_stride;
    enc->picture.cr_stride = uv_stride;
    enc->input.pts = pts;
    enc->input.flags = 0;
    // SVT-AV1 checks this against the picture it was configured for and refuses to encode ("Invalid API input
    // buffer size detected") when it is left at zero, so it is the planes' own size.
    uint32_t bytes = y_stride * enc->height + 2 * uv_stride * ((enc->height + 1) / 2);
    enc->input.n_filled_len = bytes;
    enc->input.n_alloc_len = bytes;
    EbErrorType status = svt_av1_enc_send_picture(enc->handle, &enc->input);
    if (status != EB_ErrorNone) {
        say(error, error_len, "svt_av1_enc_send_picture: 0x%x", status);
        return -1;
    }
    return 0;
}

int dimos_av1_next(dimos_av1_enc* enc, const uint8_t** data, size_t* len, int* keyframe, int64_t* pts,
                   char* error, size_t error_len) {
    EbBufferHeaderType* out = NULL;
    EbErrorType status = svt_av1_enc_get_packet(enc->handle, &out, 0);
    if (status == EB_NoErrorEmptyQueue || out == NULL) {
        return 0;
    }
    if (status != EB_ErrorNone) {
        say(error, error_len, "svt_av1_enc_get_packet: 0x%x", status);
        return -1;
    }
    enc->out = out;
    *data = out->p_buffer;
    *len = out->n_filled_len;
    *keyframe = out->pic_type == EB_AV1_KEY_PICTURE;
    *pts = out->pts;
    return 1;
}

void dimos_av1_release(dimos_av1_enc* enc) {
    if (enc->out != NULL) {
        svt_av1_enc_release_out_buffer(&enc->out);
        enc->out = NULL;
    }
}

void dimos_av1_close(dimos_av1_enc* enc) {
    if (enc == NULL) {
        return;
    }
    dimos_av1_release(enc);
    // Without an end-of-stream picture the library complains ("deinit called without sending EOS!") and the
    // tail of the stream is never flushed.
    EbBufferHeaderType eos;
    memset(&eos, 0, sizeof(eos));
    eos.size = sizeof(EbBufferHeaderType);
    eos.flags = EB_BUFFERFLAG_EOS;
    svt_av1_enc_send_picture(enc->handle, &eos);
    if (enc->header != NULL) {
        svt_av1_enc_stream_header_release(enc->header);
    }
    svt_av1_enc_deinit(enc->handle);
    svt_av1_enc_deinit_handle(enc->handle);
    free(enc);
}
