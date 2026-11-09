// Copyright 2026 Dimensional Inc.
// Licensed under the Apache License, Version 2.0.

// V4L2 capture, with Jetson hardware JPEG (VIC colour convert + NVJPG encode) when built with DIMOS_JETSON_HW.
#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct capture capture;

typedef struct {
    const uint8_t* data;  // JPEG when `jpeg` is set, else the raw frame in the opened fourcc
    size_t len;
    uint32_t stride;      // bytes per row of a raw frame
    uint32_t jpeg;
    double stamp_s;       // the driver's capture timestamp
    double encode_s;      // time the hardware spent converting and encoding this frame
} capture_frame;

// hardware: 0 never, 1 if available. Returns NULL with `error` filled on failure.
capture* capture_open(const char* device, uint32_t width, uint32_t height, uint32_t fourcc, int hardware,
                      int jpeg_quality, char* error, size_t error_len);
// 1 when frames come back JPEG-encoded by the hardware.
int capture_is_hardware(const capture* cap);
// Waits up to timeout_ms. 1: frame, valid until capture_release. 0: timed out. -1: error.
int capture_next(capture* cap, int timeout_ms, capture_frame* frame, char* error, size_t error_len);
void capture_release(capture* cap);
void capture_close(capture* cap);
