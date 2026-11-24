// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// ort_tap: the part of the LD_PRELOAD shim (built into ecan_tap.so with ecan_tap.c) that records every policy
// inference rl_deploy runs: each OrtApi::Run's input tensors (state / history / command), its outputs, the session
// (model file) it ran, CLOCK_MONOTONIC stamps around the call, a CLOCK_REALTIME stamp, and the ECAN frame count and
// last-received-frame time at that moment (the tick it belongs to). It never changes an input or an output.
// - It wraps OrtGetApiBase: the OrtApi table GetApi returns gets its CreateSession, CreateTensorWithDataAsOrtValue,
//   ReleaseValue and Run slots pointed here (the table is made writable for those stores).
// - rl_deploy's policy thread only copies into a preallocated ring (a full ring drops and counts, never waits); the
//   inputs are found by the OrtValue rl_deploy created them as (no ORT call, no allocation), the outputs' sizes are
//   asked once per session and output.
// - While $ECAN_TAP_DIR/enable exists, a low-priority writer thread on cores 1-3 writes them to
//   ort_capture_<time>.bin (layout below), its path into ort_latest_path, and its counters into ort_last_stats.
//
// File: "M20ORT1\0", then records, each u32 kind, u32 length (whole record), payload:
//   kind 1 session: u32 index, path bytes (as CreateSession got it).
//   kind 2 run: u64 seq, u32 session, u32 thread id, u64 monotonic ns before Run, u64 monotonic ns after, u64 realtime
//     ns before, u64 ECAN frames received so far, u64 realtime ns of the last one, i32 Run's status (0 ok, else 1),
//     u32 inputs, u32 outputs, then per tensor: char name[32], u32 dims, i64 dim[4], u32 floats, f32 data[floats]
//     (inputs first). A tensor the tap could not read has floats 0.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

// OrtApi slots (onnxruntime_c_api.h order, API 20)
enum {
    SLOT_CREATE_SESSION = 7,
    SLOT_RUN = 9,
    SLOT_CREATE_TENSOR_WITH_DATA = 49,
    SLOT_GET_TENSOR_MUTABLE_DATA = 51,
    SLOT_GET_TENSOR_SHAPE_ELEMENT_COUNT = 64,
    SLOT_GET_TENSOR_TYPE_AND_SHAPE = 65,
    SLOT_RELEASE_STATUS = 93,
    SLOT_RELEASE_VALUE = 96,
    SLOT_RELEASE_TENSOR_TYPE_AND_SHAPE_INFO = 99,
    SLOTS = 100,
};

typedef void* Status;  // OrtStatus*; NULL is success
typedef struct {
    const void* (*get_api)(uint32_t version);
    const char* (*get_version_string)(void);
} ApiBase;

typedef Status (*CreateSessionFn)(const void* env, const char* path, const void* options, void** out);
typedef Status (*RunFn)(void* session, const void* run_options, const char* const* input_names,
                        const void* const* inputs, size_t input_count, const char* const* output_names,
                        size_t output_count, void** outputs);
typedef Status (*CreateTensorFn)(const void* info, void* data, size_t data_bytes, const int64_t* shape,
                                 size_t shape_len, int type, void** out);
typedef Status (*GetDataFn)(void* value, void** out);
typedef Status (*GetTypeAndShapeFn)(const void* value, void** out);
typedef Status (*ElementCountFn)(const void* info, size_t* out);
typedef void (*ReleaseFn)(void* p);

static void* real[SLOTS];
static const ApiBase* (*real_get_api_base)(void);

#define MAX_SESSIONS 32
#define MAX_OUTPUTS 4
#define MAX_TENSORS 8
#define MAX_DIMS 4
#define NAME_BYTES 32
#define TENSOR_TABLE 64
#define RING 256
#define RECORD_BYTES 32768

typedef struct {
    _Atomic(void*) ptr;
    char path[256];
    _Atomic size_t output_floats[MAX_OUTPUTS];  // 0 = not asked yet
} Session;

static Session sessions[MAX_SESSIONS];
static _Atomic uint32_t session_count;

// the OrtValues rl_deploy made from its own buffers: value -> data, shape (float tensors only)
typedef struct {
    _Atomic(void*) value;
    const float* data;
    size_t floats;
    uint32_t dims;
    int64_t dim[MAX_DIMS];
} Tensor;

static Tensor tensors[TENSOR_TABLE];
static _Atomic uint32_t tensor_next;

typedef struct {
    _Atomic uint64_t ready_seq;
    uint32_t len;
    uint8_t data[RECORD_BYTES];
} Slot;

static Slot* ring;
static _Atomic uint64_t next_seq = 1;
static uint64_t consumed_seq = 1;
static _Atomic int capturing;
static _Atomic uint64_t dropped, runs_seen, runs_recorded, unreadable;
static char tap_dir[256] = "/var/opt/robot/data/ecan_capture";

// ecan_tap.c's frame counter and last frame time; 0 when the tap is built without it
extern _Atomic uint64_t ecan_tap_rx_frames __attribute__((weak));
extern _Atomic uint64_t ecan_tap_last_rx_ns __attribute__((weak));

static uint64_t ns(clockid_t clock) {
    struct timespec t;
    clock_gettime(clock, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

static int session_index(const void* session) {
    uint32_t n = atomic_load_explicit(&session_count, memory_order_acquire);
    for (uint32_t i = 0; i < n && i < MAX_SESSIONS; i++) {
        if (atomic_load_explicit(&sessions[i].ptr, memory_order_relaxed) == session) {
            return (int)i;
        }
    }
    return -1;
}

static Status tap_create_session(const void* env, const char* path, const void* options, void** out) {
    Status status = ((CreateSessionFn)real[SLOT_CREATE_SESSION])(env, path, options, out);
    if (status == NULL && out && *out) {
        uint32_t i = atomic_load(&session_count);
        if (i < MAX_SESSIONS) {
            snprintf(sessions[i].path, sizeof sessions[i].path, "%s", path ? path : "");
            atomic_store(&sessions[i].ptr, *out);
            atomic_store_explicit(&session_count, i + 1, memory_order_release);
        }
    }
    return status;
}

static Status tap_create_tensor(const void* info, void* data, size_t data_bytes, const int64_t* shape,
                                size_t shape_len, int type, void** out) {
    Status status = ((CreateTensorFn)real[SLOT_CREATE_TENSOR_WITH_DATA])(info, data, data_bytes, shape, shape_len,
                                                                          type, out);
    if (status == NULL && out && *out && type == 1 /* float */) {
        // ORT hands a released value's address out again: an entry for it is replaced, not added
        Tensor* t = NULL;
        for (int i = 0; i < TENSOR_TABLE && !t; i++) {
            if (atomic_load_explicit(&tensors[i].value, memory_order_relaxed) == *out) {
                t = &tensors[i];
            }
        }
        if (!t) {
            t = &tensors[atomic_fetch_add_explicit(&tensor_next, 1, memory_order_relaxed) % TENSOR_TABLE];
        }
        atomic_store_explicit(&t->value, NULL, memory_order_relaxed);
        t->data = data;
        t->floats = data_bytes / 4;
        t->dims = shape_len > MAX_DIMS ? MAX_DIMS : (uint32_t)shape_len;
        for (uint32_t d = 0; d < t->dims; d++) {
            t->dim[d] = shape[d];
        }
        atomic_store_explicit(&t->value, *out, memory_order_release);
    }
    return status;
}

static void tap_release_value(void* value) {
    for (int i = 0; i < TENSOR_TABLE && value; i++) {
        void* expected = value;
        atomic_compare_exchange_strong(&tensors[i].value, &expected, NULL);
    }
    ((ReleaseFn)real[SLOT_RELEASE_VALUE])(value);
}

static const Tensor* find_tensor(const void* value) {
    for (int i = 0; i < TENSOR_TABLE; i++) {
        if (atomic_load_explicit(&tensors[i].value, memory_order_acquire) == value) {
            return &tensors[i];
        }
    }
    return NULL;
}

// a tensor into the record: name, shape, data; false when it does not fit
static int put_tensor(uint8_t* r, uint32_t* used, const char* name, uint32_t dims, const int64_t* dim,
                      const float* data, size_t floats) {
    size_t need = NAME_BYTES + 4 + 8 * MAX_DIMS + 4 + 4 * floats;
    if (*used + need > RECORD_BYTES) {
        floats = 0;
        need = NAME_BYTES + 4 + 8 * MAX_DIMS + 4;
        if (*used + need > RECORD_BYTES) {
            return 0;
        }
    }
    uint8_t* p = r + *used;
    memset(p, 0, NAME_BYTES + 4 + 8 * MAX_DIMS);
    if (name) {
        strncpy((char*)p, name, NAME_BYTES - 1);
    }
    memcpy(p + NAME_BYTES, &dims, 4);
    memcpy(p + NAME_BYTES + 4, dim, 8 * (dims > MAX_DIMS ? MAX_DIMS : dims));
    uint32_t n = (uint32_t)floats;
    memcpy(p + NAME_BYTES + 4 + 8 * MAX_DIMS, &n, 4);
    if (floats) {
        memcpy(p + NAME_BYTES + 8 + 8 * MAX_DIMS, data, 4 * floats);
    }
    *used += (uint32_t)need;
    return 1;
}

// an output's float count: asked of ORT once per session and output (that call allocates), then remembered
static size_t output_floats(int session, size_t k, void* value) {
    if (session >= 0 && k < MAX_OUTPUTS) {
        size_t known = atomic_load_explicit(&sessions[session].output_floats[k], memory_order_relaxed);
        if (known) {
            return known;
        }
    }
    void* info = NULL;
    size_t count = 0;
    Status s = ((GetTypeAndShapeFn)real[SLOT_GET_TENSOR_TYPE_AND_SHAPE])(value, &info);
    if (s == NULL && info) {
        Status c = ((ElementCountFn)real[SLOT_GET_TENSOR_SHAPE_ELEMENT_COUNT])(info, &count);
        if (c) {
            ((ReleaseFn)real[SLOT_RELEASE_STATUS])(c);
            count = 0;
        }
        ((ReleaseFn)real[SLOT_RELEASE_TENSOR_TYPE_AND_SHAPE_INFO])(info);
    } else if (s) {
        ((ReleaseFn)real[SLOT_RELEASE_STATUS])(s);
    }
    if (session >= 0 && k < MAX_OUTPUTS && count) {
        atomic_store_explicit(&sessions[session].output_floats[k], count, memory_order_relaxed);
    }
    return count;
}

static Status tap_run(void* session, const void* run_options, const char* const* input_names,
                      const void* const* inputs, size_t input_count, const char* const* output_names,
                      size_t output_count, void** outputs) {
    uint64_t before = ns(CLOCK_MONOTONIC), wall = ns(CLOCK_REALTIME);
    uint64_t frames = &ecan_tap_rx_frames ? atomic_load_explicit(&ecan_tap_rx_frames, memory_order_relaxed) : 0;
    uint64_t last_rx = &ecan_tap_last_rx_ns ? atomic_load_explicit(&ecan_tap_last_rx_ns, memory_order_relaxed) : 0;
    Status status = ((RunFn)real[SLOT_RUN])(session, run_options, input_names, inputs, input_count, output_names,
                                            output_count, outputs);
    uint64_t after = ns(CLOCK_MONOTONIC);
    atomic_fetch_add_explicit(&runs_seen, 1, memory_order_relaxed);
    if (!atomic_load_explicit(&capturing, memory_order_relaxed) || ring == NULL) {
        return status;
    }
    uint64_t seq = atomic_load_explicit(&next_seq, memory_order_relaxed);
    do {
        if (seq - __atomic_load_n(&consumed_seq, __ATOMIC_ACQUIRE) >= RING) {
            atomic_fetch_add_explicit(&dropped, 1, memory_order_relaxed);
            return status;
        }
    } while (!atomic_compare_exchange_weak_explicit(&next_seq, &seq, seq + 1, memory_order_relaxed,
                                                    memory_order_relaxed));
    Slot* slot = &ring[seq % RING];
    uint8_t* r = slot->data;
    int index = session_index(session);
    uint32_t kind = 2, tid = (uint32_t)syscall(SYS_gettid), sidx = (uint32_t)index;
    int32_t ok = status == NULL ? 0 : 1;
    uint32_t ins = input_count > MAX_TENSORS ? MAX_TENSORS : (uint32_t)input_count;
    uint32_t outs = output_count > MAX_TENSORS ? MAX_TENSORS : (uint32_t)output_count;
    uint32_t used = 8;
    memcpy(r, &kind, 4);
    memcpy(r + used, &seq, 8); used += 8;
    memcpy(r + used, &sidx, 4); used += 4;
    memcpy(r + used, &tid, 4); used += 4;
    memcpy(r + used, &before, 8); used += 8;
    memcpy(r + used, &after, 8); used += 8;
    memcpy(r + used, &wall, 8); used += 8;
    memcpy(r + used, &frames, 8); used += 8;
    memcpy(r + used, &last_rx, 8); used += 8;
    memcpy(r + used, &ok, 4); used += 4;
    memcpy(r + used, &ins, 4); used += 4;
    memcpy(r + used, &outs, 4); used += 4;
    int complete = 1;
    for (uint32_t i = 0; i < ins; i++) {
        const Tensor* t = find_tensor(inputs[i]);
        if (!t) {
            atomic_fetch_add_explicit(&unreadable, 1, memory_order_relaxed);
        }
        int64_t none[MAX_DIMS] = {0};
        complete &= put_tensor(r, &used, input_names ? input_names[i] : NULL, t ? t->dims : 0, t ? t->dim : none,
                               t ? t->data : NULL, t ? t->floats : 0);
    }
    for (uint32_t k = 0; k < outs; k++) {
        float* data = NULL;
        size_t floats = 0;
        if (status == NULL && outputs && outputs[k]) {
            void* p = NULL;
            Status s = ((GetDataFn)real[SLOT_GET_TENSOR_MUTABLE_DATA])(outputs[k], &p);
            if (s == NULL) {
                data = p;
                floats = output_floats(index, k, outputs[k]);
            } else {
                ((ReleaseFn)real[SLOT_RELEASE_STATUS])(s);
            }
        }
        int64_t dim[MAX_DIMS] = {1, (int64_t)floats};
        complete &= put_tensor(r, &used, output_names ? output_names[k] : NULL, 2, dim, data, data ? floats : 0);
    }
    if (!complete) {
        atomic_fetch_add_explicit(&unreadable, 1, memory_order_relaxed);
    }
    memcpy(r + 4, &used, 4);
    slot->len = used;
    atomic_fetch_add_explicit(&runs_recorded, 1, memory_order_relaxed);
    atomic_store_explicit(&slot->ready_seq, seq, memory_order_release);
    return status;
}

// point the table's slots here: it lives in ORT's read-only data, so the page is made writable for the stores
static void patch(void** api) {
    long page = sysconf(_SC_PAGESIZE);
    uintptr_t lo = (uintptr_t)&api[SLOT_CREATE_SESSION] & ~(uintptr_t)(page - 1);
    uintptr_t hi = ((uintptr_t)&api[SLOT_RELEASE_VALUE + 1] + page - 1) & ~(uintptr_t)(page - 1);
    if (mprotect((void*)lo, hi - lo, PROT_READ | PROT_WRITE) != 0) {
        return;  // left as it is: nothing recorded
    }
    for (int i = 0; i < SLOTS; i++) {
        real[i] = api[i];
    }
    api[SLOT_CREATE_SESSION] = (void*)tap_create_session;
    api[SLOT_RUN] = (void*)tap_run;
    api[SLOT_CREATE_TENSOR_WITH_DATA] = (void*)tap_create_tensor;
    api[SLOT_RELEASE_VALUE] = (void*)tap_release_value;
    mprotect((void*)lo, hi - lo, PROT_READ);
}

static const void* (*real_get_api)(uint32_t);
static pthread_mutex_t patch_lock = PTHREAD_MUTEX_INITIALIZER;
static const void* patched_api;

static const void* tap_get_api(uint32_t version) {
    const void* api = real_get_api(version);
    if (api == NULL || version > 20) {
        return api;  // slot numbers are known for API 20 (ORT 1.20) and below
    }
    pthread_mutex_lock(&patch_lock);
    if (patched_api != api) {
        patch((void**)api);
        patched_api = api;
    }
    pthread_mutex_unlock(&patch_lock);
    return api;
}

static ApiBase tap_base;

const ApiBase* OrtGetApiBase(void) {
    if (!real_get_api_base) {
        real_get_api_base = (const ApiBase* (*)(void))dlsym(RTLD_NEXT, "OrtGetApiBase");
        if (!real_get_api_base) {
            return NULL;
        }
    }
    const ApiBase* base = real_get_api_base();
    if (!real_get_api) {
        real_get_api = base->get_api;
        tap_base.get_version_string = base->get_version_string;
        tap_base.get_api = tap_get_api;
    }
    return &tap_base;
}

static void write_stats(void) {
    char path[320];
    snprintf(path, sizeof path, "%s/ort_last_stats", tap_dir);
    FILE* f = fopen(path, "w");
    if (f) {
        fprintf(f, "runs=%llu recorded=%llu dropped=%llu unreadable=%llu sessions=%u\n",
                (unsigned long long)atomic_load(&runs_seen), (unsigned long long)atomic_load(&runs_recorded),
                (unsigned long long)atomic_load(&dropped), (unsigned long long)atomic_load(&unreadable),
                atomic_load(&session_count));
        fclose(f);
    }
}

// session records for the sessions created since the last ones written (writer thread only)
static uint32_t sessions_written;

static void write_new_sessions(FILE* f) {
    uint32_t n = atomic_load_explicit(&session_count, memory_order_acquire);
    for (uint32_t i = sessions_written; i < n && i < MAX_SESSIONS; i++) {
        uint32_t kind = 1, len = 12 + (uint32_t)strlen(sessions[i].path);
        fwrite(&kind, 4, 1, f);
        fwrite(&len, 4, 1, f);
        fwrite(&i, 4, 1, f);
        fwrite(sessions[i].path, 1, strlen(sessions[i].path), f);
    }
    sessions_written = n;
}

static FILE* open_capture(void) {
    char path[400];
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    snprintf(path, sizeof path, "%s/ort_capture_%04d%02d%02d_%02d%02d%02d.bin", tap_dir, tm.tm_year + 1900,
             tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE* f = fopen(path, "wb");
    if (!f) {
        return NULL;
    }
    fwrite("M20ORT1", 1, 8, f);
    sessions_written = 0;
    write_new_sessions(f);
    char latest[320];
    snprintf(latest, sizeof latest, "%s/ort_latest_path", tap_dir);
    FILE* l = fopen(latest, "w");
    if (l) {
        fprintf(l, "%s\n", path);
        fclose(l);
    }
    return f;
}

static void* writer_main(void* unused) {
    (void)unused;
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(1, &cpus); CPU_SET(2, &cpus); CPU_SET(3, &cpus);
    pthread_setaffinity_np(pthread_self(), sizeof cpus, &cpus);
    struct sched_param param = {0};
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    pthread_setname_np(pthread_self(), "ort_tap");
    setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 10);
    char enable_path[300];
    snprintf(enable_path, sizeof enable_path, "%s/enable", tap_dir);
    FILE* f = NULL;
    for (unsigned tick = 0;; tick++) {
        // the enable file every 50 ms (the writer wakes every 5 ms; at 50 Hz a Run lands every 20 ms)
        if (tick % 10 == 0) {
            int want = access(enable_path, F_OK) == 0;
            if (want && !f) {
                f = open_capture();
                atomic_store(&capturing, f != NULL);
            } else if (!want && f) {
                atomic_store(&capturing, 0);
            }
        }
        while (consumed_seq < atomic_load_explicit(&next_seq, memory_order_acquire)) {
            Slot* slot = &ring[consumed_seq % RING];
            if (atomic_load_explicit(&slot->ready_seq, memory_order_acquire) != consumed_seq) {
                break;  // a Run that claimed this slot is mid-copy: next wake-up
            }
            if (f) {
                write_new_sessions(f);
                fwrite(slot->data, 1, slot->len, f);
            }
            __atomic_store_n(&consumed_seq, consumed_seq + 1, __ATOMIC_RELEASE);
        }
        if (f && !atomic_load(&capturing) && atomic_load(&next_seq) == consumed_seq) {
            fclose(f);
            f = NULL;
            write_stats();
        }
        usleep(5000);
    }
    return NULL;
}

__attribute__((constructor)) static void ort_tap_init(void) {
    const char* dir = getenv("ECAN_TAP_DIR");
    if (dir && strlen(dir) < sizeof tap_dir) {
        strcpy(tap_dir, dir);
    }
    extern char* program_invocation_short_name;
    const char* only = getenv("ORT_TAP_PROCESS");
    if (strcmp(program_invocation_short_name, only ? only : "rl_deploy") != 0) {
        return;  // only rl_deploy (or the stand-in test) gets the ring and the writer thread
    }
    ring = calloc(RING, sizeof(Slot));
    if (!ring) {
        return;
    }
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    struct sched_param param = {0};
    pthread_attr_setschedparam(&attr, &param);
    pthread_t thread;
    pthread_create(&thread, &attr, writer_main, NULL);
    pthread_detach(thread);
}
