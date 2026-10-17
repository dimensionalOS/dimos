// Copyright 2026 Dimensional Inc.
// Licensed under the Apache License, Version 2.0.

#include "capture.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#ifdef DIMOS_JETSON_HW
#include <dlfcn.h>
#include <jpeglib.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

// L4T's libnvjpeg is loaded by hand: its jpeg_* symbols would otherwise collide with the libjpeg-turbo the
// CPU path links, and binding to the wrong one silently mismatches this vendor-extended struct.
static struct {
    struct jpeg_error_mgr* (*std_error)(struct jpeg_error_mgr*);
    void (*create_compress)(j_compress_ptr, int, size_t);
    void (*suppress_tables)(j_compress_ptr, boolean);
    void (*mem_dest)(j_compress_ptr, unsigned char**, unsigned long*);
    void (*set_defaults)(j_compress_ptr);
    void (*set_quality)(j_compress_ptr, int, boolean);
    void (*set_hardware)(j_compress_ptr, boolean, unsigned long, unsigned long, unsigned long);
    void (*start_compress)(j_compress_ptr, boolean);
    JDIMENSION (*write_raw_data)(j_compress_ptr, JSAMPIMAGE, JDIMENSION);
    void (*finish_compress)(j_compress_ptr);
    void (*destroy_compress)(j_compress_ptr);
} nvjpeg;

static int load_nvjpeg(char* error, size_t len);
#endif

#define BUFFERS 4

struct capture {
    int fd;
    uint32_t width, height;
    int hardware;
    int quality;
    int held;  // buffer index handed out by capture_next, or -1
    void* maps[BUFFERS];
    size_t map_lens[BUFFERS];
    uint32_t stride;
#ifdef DIMOS_JETSON_HW
    int camera_fds[BUFFERS];  // NvBufSurface dmabufs the camera fills, UYVY
    int yuv420_fd;            // the VIC's output, what NVJPG encodes
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    unsigned char* jpeg;
    unsigned long jpeg_capacity;
#endif
};

static void fail(char* error, size_t len, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(error, len, format, args);
    va_end(args);
}

#ifdef DIMOS_JETSON_HW
static int load_nvjpeg(char* error, size_t len) {
    if (nvjpeg.std_error) return 0;
    void* lib = dlopen("/usr/lib/aarch64-linux-gnu/nvidia/libnvjpeg.so", RTLD_NOW | RTLD_LOCAL);
    if (!lib) {
        fail(error, len, "no hardware JPEG: %s", dlerror());
        return -1;
    }
#define LOAD(field, name)                                       \
    *(void**)&nvjpeg.field = dlsym(lib, name);                  \
    if (!nvjpeg.field) {                                        \
        fail(error, len, "libnvjpeg has no %s", name);          \
        nvjpeg.std_error = NULL;                                \
        return -1;                                              \
    }
    LOAD(create_compress, "jpeg_CreateCompress")
    LOAD(suppress_tables, "jpeg_suppress_tables")
    LOAD(mem_dest, "jpeg_mem_dest")
    LOAD(set_defaults, "jpeg_set_defaults")
    LOAD(set_quality, "jpeg_set_quality")
    LOAD(set_hardware, "jpeg_set_hardware_acceleration_parameters_enc")
    LOAD(start_compress, "jpeg_start_compress")
    LOAD(write_raw_data, "jpeg_write_raw_data")
    LOAD(finish_compress, "jpeg_finish_compress")
    LOAD(destroy_compress, "jpeg_destroy_compress")
    LOAD(std_error, "jpeg_std_error")
#undef LOAD
    return 0;
}
#endif

static int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

static int set_format(capture* cap, uint32_t fourcc, char* error, size_t len) {
    struct v4l2_format fmt = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    fmt.fmt.pix.width = cap->width;
    fmt.fmt.pix.height = cap->height;
    fmt.fmt.pix.pixelformat = fourcc;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(cap->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fail(error, len, "VIDIOC_S_FMT: %s", strerror(errno));
        return -1;
    }
    if (fmt.fmt.pix.width != cap->width || fmt.fmt.pix.height != cap->height) {
        fail(error, len, "driver gave %ux%u, not %ux%u", fmt.fmt.pix.width, fmt.fmt.pix.height, cap->width,
             cap->height);
        return -1;
    }
    cap->stride = fmt.fmt.pix.bytesperline;
    return 0;
}

static int start_mmap(capture* cap, char* error, size_t len) {
    struct v4l2_requestbuffers req = {.count = BUFFERS, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    if (xioctl(cap->fd, VIDIOC_REQBUFS, &req) < 0 || req.count != BUFFERS) {
        fail(error, len, "VIDIOC_REQBUFS mmap: %s", strerror(errno));
        return -1;
    }
    for (unsigned i = 0; i < BUFFERS; i++) {
        struct v4l2_buffer buf = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i};
        if (xioctl(cap->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            fail(error, len, "VIDIOC_QUERYBUF: %s", strerror(errno));
            return -1;
        }
        cap->maps[i] = mmap(NULL, buf.length, PROT_READ, MAP_SHARED, cap->fd, buf.m.offset);
        if (cap->maps[i] == MAP_FAILED) {
            cap->maps[i] = NULL;
            fail(error, len, "mmap: %s", strerror(errno));
            return -1;
        }
        cap->map_lens[i] = buf.length;
        if (xioctl(cap->fd, VIDIOC_QBUF, &buf) < 0) {
            fail(error, len, "VIDIOC_QBUF: %s", strerror(errno));
            return -1;
        }
    }
    return 0;
}

#ifdef DIMOS_JETSON_HW
static int allocate(uint32_t width, uint32_t height, NvBufSurfaceColorFormat format, NvBufSurfaceTag tag) {
    NvBufSurfaceAllocateParams params;
    memset(&params, 0, sizeof(params));
    params.params.width = width;
    params.params.height = height;
    params.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    params.params.layout = NVBUF_LAYOUT_PITCH;
    params.params.colorFormat = format;
    params.memtag = tag;
    NvBufSurface* surface = NULL;
    if (NvBufSurfaceAllocate(&surface, 1, &params) != 0 || surface == NULL) {
        return -1;
    }
    surface->numFilled = 1;
    return (int)surface->surfaceList[0].bufferDesc;
}

static void destroy(int fd) {
    NvBufSurface* surface = NULL;
    if (fd >= 0 && NvBufSurfaceFromFd(fd, (void**)&surface) == 0) {
        NvBufSurfaceDestroy(surface);
    }
}

// Only UYVY-family cameras: the VIC converts them, NVJPG encodes YUV420.
static int start_hardware(capture* cap, uint32_t fourcc, char* error, size_t len) {
    NvBufSurfaceColorFormat format;
    switch (fourcc) {
        case V4L2_PIX_FMT_UYVY: format = NVBUF_COLOR_FORMAT_UYVY; break;
        case V4L2_PIX_FMT_YUYV: format = NVBUF_COLOR_FORMAT_YUYV; break;
        case V4L2_PIX_FMT_VYUY: format = NVBUF_COLOR_FORMAT_VYUY; break;
        case V4L2_PIX_FMT_YVYU: format = NVBUF_COLOR_FORMAT_YVYU; break;
        default: fail(error, len, "no hardware path for this pixel format"); return -1;
    }
    for (unsigned i = 0; i < BUFFERS; i++) {
        cap->camera_fds[i] = allocate(cap->width, cap->height, format, NvBufSurfaceTag_CAMERA);
        if (cap->camera_fds[i] < 0) {
            fail(error, len, "NvBufSurfaceAllocate for the camera failed");
            return -1;
        }
    }
    cap->yuv420_fd = allocate(cap->width, cap->height, NVBUF_COLOR_FORMAT_YUV420, NvBufSurfaceTag_NONE);
    if (cap->yuv420_fd < 0) {
        fail(error, len, "NvBufSurfaceAllocate for the encoder failed");
        return -1;
    }
    struct v4l2_requestbuffers req = {.count = BUFFERS, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_DMABUF};
    if (xioctl(cap->fd, VIDIOC_REQBUFS, &req) < 0 || req.count != BUFFERS) {
        fail(error, len, "VIDIOC_REQBUFS dmabuf: %s", strerror(errno));
        return -1;
    }
    for (unsigned i = 0; i < BUFFERS; i++) {
        struct v4l2_buffer buf = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_DMABUF, .index = i};
        if (xioctl(cap->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            fail(error, len, "VIDIOC_QUERYBUF dmabuf: %s", strerror(errno));
            return -1;
        }
        buf.m.fd = cap->camera_fds[i];
        if (xioctl(cap->fd, VIDIOC_QBUF, &buf) < 0) {
            fail(error, len, "VIDIOC_QBUF dmabuf: %s", strerror(errno));
            return -1;
        }
    }
    if (load_nvjpeg(error, len) < 0) return -1;
    memset(&cap->cinfo, 0, sizeof(cap->cinfo));
    cap->cinfo.err = nvjpeg.std_error(&cap->jerr);
    nvjpeg.create_compress(&cap->cinfo, JPEG_LIB_VERSION, sizeof(cap->cinfo));
    nvjpeg.suppress_tables(&cap->cinfo, TRUE);
    cap->jpeg_capacity = (unsigned long)cap->width * cap->height * 3 / 2;
    cap->jpeg = malloc(cap->jpeg_capacity);
    return cap->jpeg ? 0 : -1;
}

static int encode(capture* cap, int camera_fd, size_t* out_len, char* error, size_t len) {
    NvBufSurface *source = NULL, *target = NULL;
    NvBufSurfaceFromFd(camera_fd, (void**)&source);
    NvBufSurfaceFromFd(cap->yuv420_fd, (void**)&target);
    NvBufSurfTransformRect rect = {0, 0, cap->width, cap->height};
    NvBufSurfTransformParams params;
    memset(&params, 0, sizeof(params));
    params.transform_flag = NVBUFSURF_TRANSFORM_FILTER;
    params.transform_filter = NvBufSurfTransformInter_Nearest;
    params.src_rect = &rect;
    params.dst_rect = &rect;
    if (NvBufSurfTransform(source, target, &params) != NvBufSurfTransformError_Success) {
        fail(error, len, "NvBufSurfTransform UYVY -> YUV420 failed");
        return -1;
    }
    // jpeg_mem_dest grows the buffer in place when a frame outgrows it.
    unsigned long size = cap->jpeg_capacity;
    nvjpeg.mem_dest(&cap->cinfo, &cap->jpeg, &size);
    cap->cinfo.fd = cap->yuv420_fd;
    cap->cinfo.IsVendorbuf = TRUE;
    cap->cinfo.raw_data_in = TRUE;
    cap->cinfo.in_color_space = JCS_YCbCr;
    nvjpeg.set_defaults(&cap->cinfo);
    nvjpeg.set_quality(&cap->cinfo, cap->quality, TRUE);
    nvjpeg.set_hardware(&cap->cinfo, TRUE, size, 0, 0);
    cap->cinfo.in_color_space = JCS_YCbCr;
    nvjpeg.start_compress(&cap->cinfo, 0);
    if (cap->cinfo.err->msg_code) {
        char message[JMSG_LENGTH_MAX];
        cap->cinfo.err->format_message((j_common_ptr)&cap->cinfo, message);
        fail(error, len, "jpeg_start_compress: %s", message);
        return -1;
    }
    nvjpeg.write_raw_data(&cap->cinfo, NULL, 0);
    nvjpeg.finish_compress(&cap->cinfo);
    if (size > cap->jpeg_capacity) {
        cap->jpeg_capacity = size;
    }
    *out_len = size;
    return 0;
}
#endif

capture* capture_open(const char* device, uint32_t width, uint32_t height, uint32_t fourcc, int hardware,
                      int jpeg_quality, char* error, size_t error_len) {
    capture* cap = calloc(1, sizeof(capture));
    if (!cap) {
        fail(error, error_len, "out of memory");
        return NULL;
    }
    cap->width = width;
    cap->height = height;
    cap->quality = jpeg_quality;
    cap->held = -1;
#ifdef DIMOS_JETSON_HW
    for (unsigned i = 0; i < BUFFERS; i++) cap->camera_fds[i] = -1;
    cap->yuv420_fd = -1;
#endif
    cap->fd = open(device, O_RDWR | O_NONBLOCK);
    if (cap->fd < 0) {
        fail(error, error_len, "open %s: %s", device, strerror(errno));
        free(cap);
        return NULL;
    }
    if (set_format(cap, fourcc, error, error_len) < 0) {
        capture_close(cap);
        return NULL;
    }
    int started = -1;
#ifdef DIMOS_JETSON_HW
    if (hardware && jpeg_quality > 0) {
        started = start_hardware(cap, fourcc, error, error_len);
        cap->hardware = started == 0;
        if (started < 0) {
            // Fall back to CPU capture on a fresh fd; the failed REQBUFS may have left this one claimed.
            capture_close(cap);
            return capture_open(device, width, height, fourcc, 0, jpeg_quality, error, error_len);
        }
    }
#else
    (void)hardware;
#endif
    if (started < 0 && start_mmap(cap, error, error_len) < 0) {
        capture_close(cap);
        return NULL;
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(cap->fd, VIDIOC_STREAMON, &type) < 0) {
        fail(error, error_len, "VIDIOC_STREAMON: %s", strerror(errno));
        capture_close(cap);
        return NULL;
    }
    return cap;
}

int capture_is_hardware(const capture* cap) { return cap->hardware; }

int capture_next(capture* cap, int timeout_ms, capture_frame* frame, char* error, size_t error_len) {
    capture_release(cap);
    struct pollfd pfd = {.fd = cap->fd, .events = POLLIN};
    int ready = poll(&pfd, 1, timeout_ms);
    if (ready == 0) return 0;
    if (ready < 0) {
        if (errno == EINTR) return 0;
        fail(error, error_len, "poll: %s", strerror(errno));
        return -1;
    }
    struct v4l2_buffer buf = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                              .memory = cap->hardware ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP};
    if (xioctl(cap->fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) return 0;
        fail(error, error_len, "VIDIOC_DQBUF: %s", strerror(errno));
        return -1;
    }
    cap->held = (int)buf.index;
    frame->stamp_s = buf.timestamp.tv_sec + buf.timestamp.tv_usec * 1e-6;
    frame->stride = cap->stride;
    frame->encode_s = 0;
#ifdef DIMOS_JETSON_HW
    if (cap->hardware) {
        size_t size = 0;
        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);
        if (encode(cap, cap->camera_fds[buf.index], &size, error, error_len) < 0) return -1;
        clock_gettime(CLOCK_MONOTONIC, &end);
        frame->encode_s = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) * 1e-9;
        frame->data = cap->jpeg;
        frame->len = size;
        frame->jpeg = 1;
        return 1;
    }
#endif
    frame->data = cap->maps[buf.index];
    frame->len = buf.bytesused;
    frame->jpeg = 0;
    return 1;
}

void capture_release(capture* cap) {
    if (cap->held < 0) return;
    struct v4l2_buffer buf = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                              .memory = cap->hardware ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP,
                              .index = (uint32_t)cap->held};
#ifdef DIMOS_JETSON_HW
    if (cap->hardware) buf.m.fd = cap->camera_fds[cap->held];
#endif
    xioctl(cap->fd, VIDIOC_QBUF, &buf);
    cap->held = -1;
}

void capture_close(capture* cap) {
    if (!cap) return;
    if (cap->fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(cap->fd, VIDIOC_STREAMOFF, &type);
    }
    for (unsigned i = 0; i < BUFFERS; i++) {
        if (cap->maps[i]) munmap(cap->maps[i], cap->map_lens[i]);
    }
#ifdef DIMOS_JETSON_HW
    if (cap->hardware) {
        nvjpeg.destroy_compress(&cap->cinfo);
        free(cap->jpeg);
    }
    for (unsigned i = 0; i < BUFFERS; i++) destroy(cap->camera_fds[i]);
    destroy(cap->yuv420_fd);
#endif
    if (cap->fd >= 0) close(cap->fd);
    free(cap);
}
