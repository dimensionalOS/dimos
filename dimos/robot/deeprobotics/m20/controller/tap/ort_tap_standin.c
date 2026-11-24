// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// ort_tap_standin: rl_deploy's inference pattern without rl_deploy, to test ort_tap offline. Like rl_deploy it gets the
// OrtApi through OrtGetApiBase()->GetApi(20), creates its sessions (1 intra-op thread, all optimizations), and each
// step wraps its own buffers with CreateTensorWithDataAsOrtValue and calls Run, at 50 Hz on SCHED_FIFO when allowed.
// It writes what it fed and got (OUT: per Run u32 session, the inputs' floats, the outputs' floats) and prints the
// Run latency (monotonic around the call) for comparing with and without the tap preloaded.
//
//     ort_tap_standin LIBONNXRUNTIME FRAMES_MODEL SEQUENCE_MODEL STEPS OUT
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef void* Status;
typedef struct {
    const void* (*get_api)(uint32_t version);
    const char* (*get_version_string)(void);
} ApiBase;
static void** api;
// the OrtApi entries used, typed (slot numbers: onnxruntime_c_api.h, API 20)
#define FN(slot, ...) ((Status(*)(__VA_ARGS__))api[slot])
#define CREATE_ENV FN(3, int, const char*, void**)
#define CREATE_SESSION FN(7, void*, const char*, void*, void**)
#define RUN FN(9, void*, void*, const char* const*, void* const*, size_t, const char* const*, size_t, void**)
#define CREATE_SESSION_OPTIONS FN(10, void**)
#define SET_OPTIMIZATION FN(23, void*, int)
#define SET_THREADS FN(24, void*, int)
#define CREATE_TENSOR FN(49, void*, void*, size_t, const int64_t*, size_t, int, void**)
#define GET_DATA FN(51, void*, float**)
#define CREATE_CPU_MEMORY_INFO FN(69, int, int, void**)

static void check(Status s, const char* what) {
    if (s) {
        fprintf(stderr, "%s: %s\n", what, ((const char* (*)(Status))api[2])(s));
        exit(1);
    }
}

static uint64_t mono(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

static int cmp(const void* a, const void* b) {
    uint64_t x = *(const uint64_t*)a, y = *(const uint64_t*)b;
    return x < y ? -1 : x > y;
}

int main(int argc, char** argv) {
    if (argc != 6) {
        fprintf(stderr, "usage: %s LIBONNXRUNTIME FRAMES_MODEL SEQUENCE_MODEL STEPS OUT\n", argv[0]);
        return 2;
    }
    // the library loaded globally first, as rl_deploy has it linked; the preloaded tap's OrtGetApiBase comes first
    if (!dlopen(argv[1], RTLD_NOW | RTLD_GLOBAL)) {
        fprintf(stderr, "%s\n", dlerror());
        return 1;
    }
    const ApiBase* (*get_base)(void) = (const ApiBase* (*)(void))dlsym(RTLD_DEFAULT, "OrtGetApiBase");
    api = (void**)get_base()->get_api(20);
    void *env, *options, *frames, *sequence, *memory;
    check(CREATE_ENV(2, "standin", &env), "CreateEnv");
    check(CREATE_SESSION_OPTIONS(&options), "CreateSessionOptions");
    check(SET_THREADS(options, 1), "SetIntraOpNumThreads");
    check(SET_OPTIMIZATION(options, 99), "SetSessionGraphOptimizationLevel");
    check(CREATE_SESSION(env, argv[2], options, &frames), "CreateSession frames");
    check(CREATE_SESSION(env, argv[3], options, &sequence), "CreateSession sequence");
    check(CREATE_CPU_MEMORY_INFO(1, 0, &memory), "CreateCpuMemoryInfo");
    int steps = atoi(argv[4]);
    FILE* out = fopen(argv[5], "wb");
    struct sched_param param = {.sched_priority = 80};
    int rt = sched_setscheduler(0, SCHED_FIFO, &param) == 0;
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(0, &cpus);
    sched_setaffinity(0, sizeof cpus, &cpus);
    static float state1[318], state2[36 * 53], command[21], uncertain[1];
    uint64_t* latency = calloc((size_t)steps, sizeof(uint64_t));
    uint32_t seed = 12345;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (int k = 0; k < steps; k++) {
        int seq = k % 2;  // alternate the two kinds of session
        float* bufs[3] = {seq ? state2 : state1, command, uncertain};
        size_t floats[3] = {seq ? 36 * 53 : 318, 21, 1};
        for (int b = 0; b < (seq ? 3 : 1); b++) {
            for (size_t i = 0; i < floats[b]; i++) {
                seed = seed * 1664525u + 1013904223u;
                bufs[b][i] = ((float)(seed >> 8) / 16777216.0f - 0.5f) * 2.0f;
            }
        }
        int64_t s1[2] = {1, 318}, s2[3] = {1, 36, 53}, s3[2] = {1, 21}, s4[2] = {1, 1};
        void* values[3] = {0};
        const char* in_names[3] = {"state", "command", "uncertain"};
        const char* out_names[2] = {"output", "vel"};
        void* outputs[2] = {0};
        size_t n_in = seq ? 3 : 1, n_out = seq ? 1 : 2;
        if (seq) {
            check(CREATE_TENSOR(memory, state2, sizeof state2, s2, 3, 1, &values[0]), "CreateTensor");
            check(CREATE_TENSOR(memory, command, sizeof command, s3, 2, 1, &values[1]), "CreateTensor");
            check(CREATE_TENSOR(memory, uncertain, sizeof uncertain, s4, 2, 1, &values[2]), "CreateTensor");
        } else {
            check(CREATE_TENSOR(memory, state1, sizeof state1, s1, 2, 1, &values[0]), "CreateTensor");
        }
        uint64_t t0 = mono();
        check(RUN(seq ? sequence : frames, NULL, in_names, values, n_in, out_names, n_out, outputs), "Run");
        latency[k] = mono() - t0;
        uint32_t which = (uint32_t)seq;
        fwrite(&which, 4, 1, out);
        for (size_t b = 0; b < n_in; b++) {
            fwrite(bufs[b], 4, floats[b], out);
        }
        size_t out_floats[2] = {16, 3};
        for (size_t o = 0; o < n_out; o++) {
            float* data;
            check(GET_DATA(outputs[o], &data), "GetTensorMutableData");
            fwrite(data, 4, out_floats[o], out);
            ((void (*)(void*))api[96])(outputs[o]);
        }
        for (size_t b = 0; b < n_in; b++) {
            ((void (*)(void*))api[96])(values[b]);
        }
        next.tv_nsec += 20000000;
        if (next.tv_nsec >= 1000000000) {
            next.tv_nsec -= 1000000000;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    fclose(out);
    // per session kind: median, p99, max Run latency (us), the first 10 steps (warm-up) left out
    for (int seq = 0; seq < 2; seq++) {
        uint64_t* v = calloc((size_t)steps, sizeof(uint64_t));
        int n = 0;
        for (int k = 10; k < steps; k++) {
            if (k % 2 == seq) {
                v[n++] = latency[k];
            }
        }
        qsort(v, (size_t)n, sizeof v[0], cmp);
        printf("%s rt=%d runs=%d run_us median %.1f p99 %.1f max %.1f\n", seq ? "sequence" : "frames", rt, n,
               v[n / 2] / 1e3, v[n * 99 / 100] / 1e3, v[n - 1] / 1e3);
        free(v);
    }
    return 0;
}
