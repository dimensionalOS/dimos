// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// ecan_tap.so: LD_PRELOAD shim for the vendor rl_deploy that copies every frame on its AF_PACKET (ECAN motor bus)
// socket, both ways, out of the process. It never sends or changes a frame.
// - pcapng: while $ECAN_TAP_DIR/enable exists, into capture_<time>.pcapng (one file per enable).
// - live: while the unix datagram socket $ECAN_TAP_SOCKET exists (a reader bound it), as datagrams of records
//   (32-byte header + Ethernet frame, layout in the m20-ecan crate's `tap` module).
// Charge-enable test controls (off unless their file exists; the writer thread polls them, the RT threads read atomics):
// - $ECAN_TAP_DIR/drop_charge_enable present: rl_deploy's 0xb3 charge-enable frames are not sent (reported as sent);
//   every other frame passes untouched.
// - $ECAN_TAP_DIR/inject_charge_enable appears: it is removed, and the next 3 joint commands go out as 0xb3 frames with
//   their sequence numbers (what rl_deploy itself does), only while every joint's kp and kd in them are zero; a command
//   with any gain cancels the request.
// rl_deploy's real-time threads only memcpy into a preallocated ring (a full ring drops, never waits); a low-priority
// writer thread on cores 1-3 does all file and socket I/O, and drops a datagram the reader is not ready for.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define SLOT_COUNT 8192
#define MAX_FD 4096
#define SLOT_BYTES 1536
#define RECORD_HEADER 32

typedef struct {
    _Atomic uint64_t ready_seq;
    uint64_t realtime_ns;
    uint32_t len;
    uint32_t direction;  // 1 = received by rl_deploy, 2 = sent by rl_deploy (pcapng epb_flags convention)
    uint8_t data[SLOT_BYTES];
} Slot;

static Slot* slots;
static _Atomic uint64_t next_seq = 1;
static _Atomic uint64_t dropped = 0;
static _Atomic uint64_t ring_dropped_total = 0;
static uint64_t consumed_seq = 1;
static _Atomic int capturing = 0;
static _Atomic uint8_t packet_fds[MAX_FD];
static _Atomic int drop_charge_enable = 0;
static _Atomic int inject_pending = 0;
static _Atomic uint64_t dropped_charge_enable = 0;
static _Atomic uint64_t injected_charge_enable = 0;

#define ETH_LEN 14
#define JOINT_UNITS 16
#define JOINT_UNIT_BYTES 24

static int is_charge_enable(const uint8_t* f, size_t len) {
    return len >= ETH_LEN + 10 && f[12] == 0x44 && f[13] == 0x52 && f[ETH_LEN + 5] == 0xb3;
}

// a joint command (type 0x89 out) whose 16 units all carry kp = kd = 0
static int is_passive_joint_command(const uint8_t* f, size_t len) {
    if (len < ETH_LEN + 10 + JOINT_UNITS * JOINT_UNIT_BYTES || f[12] != 0x44 || f[13] != 0x52 || f[ETH_LEN + 5] != 0x89) {
        return 0;
    }
    for (int i = 0; i < JOINT_UNITS; i++) {
        const uint8_t* u = f + ETH_LEN + 10 + i * JOINT_UNIT_BYTES;
        if (u[16] | u[17] | u[18] | u[19]) {
            return 0;
        }
    }
    return 1;
}
static ssize_t (*real_sendto)(int, const void*, size_t, int, const struct sockaddr*, socklen_t);
static ssize_t (*real_recvfrom)(int, void*, size_t, int, struct sockaddr*, socklen_t*);
static int (*real_socket)(int, int, int);
static char tap_dir[256] = "/var/opt/robot/data/ecan_capture";
static char live_path[sizeof(((struct sockaddr_un*)0)->sun_path)];
// writer wake-up period: the live stream's worst-case added latency
static long poll_us = 1000;
// how often the writer checks for the enable file and the live socket
static long control_poll_us = 50000;
// the most frame bytes one live datagram carries
static size_t datagram_bytes = 32768;

static long env_long(const char* name, long fallback, long low, long high) {
    const char* text = getenv(name);
    if (!text) {
        return fallback;
    }
    char* end;
    long value = strtol(text, &end, 10);
    return (*end == 0 && value >= low && value <= high) ? value : fallback;
}

static void capture(const void* buf, size_t len, uint32_t direction) {
    if (!atomic_load_explicit(&capturing, memory_order_relaxed) || slots == NULL) {
        return;
    }
    uint64_t seq = atomic_load_explicit(&next_seq, memory_order_relaxed);
    do {
        if (seq - __atomic_load_n(&consumed_seq, __ATOMIC_ACQUIRE) >= SLOT_COUNT) {
            atomic_fetch_add_explicit(&dropped, 1, memory_order_relaxed);
            atomic_fetch_add_explicit(&ring_dropped_total, 1, memory_order_relaxed);
            return;
        }
    } while (!atomic_compare_exchange_weak_explicit(&next_seq, &seq, seq + 1, memory_order_relaxed, memory_order_relaxed));
    Slot* slot = &slots[seq % SLOT_COUNT];
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    slot->realtime_ns = (uint64_t)now.tv_sec * 1000000000ull + (uint64_t)now.tv_nsec;
    slot->len = len > SLOT_BYTES ? SLOT_BYTES : (uint32_t)len;
    slot->direction = direction;
    memcpy(slot->data, buf, slot->len);
    atomic_store_explicit(&slot->ready_seq, seq, memory_order_release);
}

static int is_packet_fd(int fd) {
    return fd >= 0 && fd < MAX_FD && atomic_load_explicit(&packet_fds[fd], memory_order_relaxed);
}

int socket(int domain, int type, int protocol) {
    if (!real_socket) {
        real_socket = dlsym(RTLD_NEXT, "socket");
    }
    int fd = real_socket(domain, type, protocol);
    if (fd >= 0 && fd < MAX_FD) {
        atomic_store(&packet_fds[fd], domain == AF_PACKET);
    }
    return fd;
}

ssize_t sendto(int fd, const void* buf, size_t len, int flags, const struct sockaddr* addr, socklen_t addrlen) {
    if (!real_sendto) {
        real_sendto = dlsym(RTLD_NEXT, "sendto");
    }
    if (is_packet_fd(fd)) {
        const uint8_t* f = buf;
        if (atomic_load_explicit(&drop_charge_enable, memory_order_relaxed) && is_charge_enable(f, len)) {
            atomic_fetch_add_explicit(&dropped_charge_enable, 1, memory_order_relaxed);
            return (ssize_t)len;
        }
        if (atomic_load_explicit(&inject_pending, memory_order_relaxed) > 0 && len >= ETH_LEN + 10 &&
            f[ETH_LEN + 5] == 0x89 && !is_passive_joint_command(f, len)) {
            atomic_store_explicit(&inject_pending, 0, memory_order_relaxed);  // gains on: cancel, never inject later
        }
        if (atomic_load_explicit(&inject_pending, memory_order_relaxed) > 0 && is_passive_joint_command(f, len)) {
            // header-only 0xb3 frame: rl_deploy's Ethernet header and sequence, magic 0, flags 0, code 0x11
            uint8_t enable[ETH_LEN + 10] = {0};
            memcpy(enable, f, ETH_LEN);
            enable[ETH_LEN + 2] = f[ETH_LEN + 2];
            enable[ETH_LEN + 4] = 0x11;
            enable[ETH_LEN + 5] = 0xb3;
            ssize_t sent = real_sendto(fd, enable, sizeof enable, flags, addr, addrlen);
            if (sent > 0) {
                atomic_fetch_sub_explicit(&inject_pending, 1, memory_order_relaxed);
                atomic_fetch_add_explicit(&injected_charge_enable, 1, memory_order_relaxed);
                capture(enable, (size_t)sent, 2);
                return (ssize_t)len;  // rl_deploy sees its command as sent, as in its own charge-enable cycles
            }
        }
    }
    ssize_t result = real_sendto(fd, buf, len, flags, addr, addrlen);
    if (result > 0 && is_packet_fd(fd)) {
        capture(buf, (size_t)result, 2);
    }
    return result;
}

ssize_t recvfrom(int fd, void* buf, size_t len, int flags, struct sockaddr* addr, socklen_t* addrlen) {
    if (!real_recvfrom) {
        real_recvfrom = dlsym(RTLD_NEXT, "recvfrom");
    }
    ssize_t result = real_recvfrom(fd, buf, len, flags, addr, addrlen);
    if (result > 0 && is_packet_fd(fd)) {
        capture(buf, (size_t)result, 1);
    }
    return result;
}

static void put32(FILE* f, uint32_t v) { fwrite(&v, 4, 1, f); }
static void put16(FILE* f, uint16_t v) { fwrite(&v, 2, 1, f); }

static FILE* open_capture(void) {
    char path[400];
    time_t t = time(NULL);
    struct tm tmv;
    localtime_r(&t, &tmv);
    char stamp[64];
    strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", &tmv);
    snprintf(path, sizeof path, "%s/capture_%s.pcapng", tap_dir, stamp);
    FILE* f = fopen(path, "wb");
    if (!f) {
        return NULL;
    }
    setvbuf(f, NULL, _IOFBF, 1 << 20);
    // section header block
    put32(f, 0x0A0D0D0A); put32(f, 28); put32(f, 0x1A2B3C4D); put16(f, 1); put16(f, 0);
    put32(f, 0xFFFFFFFF); put32(f, 0xFFFFFFFF); put32(f, 28);
    // interface description block: linktype ethernet, if_tsresol = 9 (nanoseconds), if_name = eth1
    put32(f, 1); put32(f, 40); put16(f, 1); put16(f, 0); put32(f, 65535);
    put16(f, 9); put16(f, 1); fputc(9, f); fputc(0, f); fputc(0, f); fputc(0, f);
    put16(f, 2); put16(f, 4); fwrite("eth1", 4, 1, f);
    put16(f, 0); put16(f, 0);
    put32(f, 40);
    char latest[400];
    snprintf(latest, sizeof latest, "%s/latest_path", tap_dir);
    FILE* lf = fopen(latest, "w");
    if (lf) {
        fprintf(lf, "%s\n", path);
        fclose(lf);
    }
    return f;
}

static void write_epb(FILE* f, const Slot* slot) {
    uint32_t padded = (slot->len + 3u) & ~3u;
    uint32_t total = 32 + padded + 12;  // header 28 + data + epb_flags option (8) + end-of-options (4) + trailing length (4)
    put32(f, 6); put32(f, total); put32(f, 0);
    put32(f, (uint32_t)(slot->realtime_ns >> 32)); put32(f, (uint32_t)slot->realtime_ns);
    put32(f, slot->len); put32(f, slot->len);
    fwrite(slot->data, 1, slot->len, f);
    static const uint8_t zeros[4] = {0};
    fwrite(zeros, 1, padded - slot->len, f);
    put16(f, 2); put16(f, 4); put32(f, slot->direction);
    put16(f, 0); put16(f, 0);
    put32(f, total);
}

typedef struct {
    int fd;
    struct sockaddr_un address;
    uint8_t* buffer;
    size_t used;
    uint32_t send_dropped;
} Live;

static void live_flush(Live* live) {
    if (live->used == 0) {
        return;
    }
    ssize_t sent = sendto(live->fd, live->buffer, live->used, MSG_DONTWAIT | MSG_NOSIGNAL,
                          (struct sockaddr*)&live->address, sizeof live->address);
    if (sent < 0) {
        live->send_dropped++;
        if (errno == ECONNREFUSED) {
            unlink(live_path);  // the reader died without removing its socket; stop streaming until one binds again
        }
    }
    live->used = 0;
}

static void live_add(Live* live, const Slot* slot) {
    if (live->used + RECORD_HEADER + slot->len > datagram_bytes + RECORD_HEADER + SLOT_BYTES) {
        live_flush(live);
    }
    uint8_t* r = live->buffer + live->used;
    uint16_t version = 1, len = (uint16_t)slot->len;
    uint32_t ring = (uint32_t)atomic_load_explicit(&ring_dropped_total, memory_order_relaxed), reserved = 0;
    memcpy(r, "ETAP", 4);
    memcpy(r + 4, &version, 2);
    memcpy(r + 6, &len, 2);
    memcpy(r + 8, &slot->direction, 4);
    memcpy(r + 12, &ring, 4);
    memcpy(r + 16, &live->send_dropped, 4);
    memcpy(r + 20, &reserved, 4);
    memcpy(r + 24, &slot->realtime_ns, 8);
    memcpy(r + RECORD_HEADER, slot->data, slot->len);
    live->used += RECORD_HEADER + slot->len;
    if (live->used >= datagram_bytes) {
        live_flush(live);
    }
}

// counters of the charge-enable controls, for the test log
static void write_control_stats(void) {
    static uint64_t last = UINT64_MAX;
    uint64_t now = atomic_load(&dropped_charge_enable) * 4 + atomic_load(&injected_charge_enable) * 1000003u +
                   (uint64_t)atomic_load(&drop_charge_enable) + (uint64_t)atomic_load(&inject_pending) * 7919u;
    if (now == last) {
        return;  // only when something changed
    }
    last = now;
    char path[320];
    snprintf(path, sizeof path, "%s/charge_control_stats", tap_dir);
    FILE* f = fopen(path, "w");
    if (f) {
        fprintf(f, "drop=%d dropped=%llu injected=%llu pending=%d\n", atomic_load(&drop_charge_enable),
                (unsigned long long)atomic_load(&dropped_charge_enable),
                (unsigned long long)atomic_load(&injected_charge_enable), atomic_load(&inject_pending));
        fclose(f);
    }
}

static void* writer_main(void* unused) {
    (void)unused;
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(1, &cpus); CPU_SET(2, &cpus); CPU_SET(3, &cpus);
    pthread_setaffinity_np(pthread_self(), sizeof cpus, &cpus);
    struct sched_param param = {0};
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    pthread_setname_np(pthread_self(), "ecan_tap");
    setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 10);
    char enable_path[300], drop_path[300], inject_path[300];
    snprintf(enable_path, sizeof enable_path, "%s/enable", tap_dir);
    snprintf(drop_path, sizeof drop_path, "%s/drop_charge_enable", tap_dir);
    snprintf(inject_path, sizeof inject_path, "%s/inject_charge_enable", tap_dir);
    Live live = {.fd = -1, .buffer = malloc(datagram_bytes + 2 * (RECORD_HEADER + SLOT_BYTES))};
    live.address.sun_family = AF_UNIX;
    memcpy(live.address.sun_path, live_path, sizeof live.address.sun_path);
    int live_on = 0;
    int close_file = 0;
    FILE* out = NULL;
    uint64_t written = 0;
    // after capturing stops: drain every slot claimed before that (each claimed slot is always published)
    uint64_t drain_until = 0;
    long ticks_per_check = control_poll_us / poll_us > 0 ? control_poll_us / poll_us : 1;
    long poll_ticks = ticks_per_check;
    for (;;) {
        if (++poll_ticks >= ticks_per_check) {
            poll_ticks = 0;
            struct stat st;
            int want_file = stat(enable_path, &st) == 0;
            int drop = stat(drop_path, &st) == 0;
            atomic_store(&drop_charge_enable, drop);
            if (unlink(inject_path) == 0) {
                atomic_store(&inject_pending, 3);
            }
            write_control_stats();
            int reader = live_path[0] && live.buffer && stat(live_path, &st) == 0 && S_ISSOCK(st.st_mode);
            if (reader && live.fd < 0) {
                live.fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
            }
            live_on = reader && live.fd >= 0;
            if (want_file && !out) {
                out = open_capture();
                written = 0;
                atomic_store(&dropped, 0);
            }
            close_file = !want_file && out;
            int on = (want_file && out) || live_on;
            if (on && !atomic_load(&capturing)) {
                __atomic_store_n(&consumed_seq, atomic_load(&next_seq), __ATOMIC_RELEASE);
                atomic_store(&capturing, 1);
            } else if (!on && atomic_load(&capturing)) {
                atomic_store(&capturing, 0);
                drain_until = atomic_load(&next_seq);
            }
        }
        for (;;) {
            Slot* slot = &slots[consumed_seq % SLOT_COUNT];
            if (atomic_load_explicit(&slot->ready_seq, memory_order_acquire) != consumed_seq) {
                if (consumed_seq < drain_until) {
                    sched_yield();  // a capture() that claimed this slot is mid-memcpy
                    continue;
                }
                break;
            }
            if (slot->len > 0) {
                if (out) {
                    write_epb(out, slot);
                    written++;
                }
                if (live_on) {
                    live_add(&live, slot);
                }
            }
            __atomic_store_n(&consumed_seq, consumed_seq + 1, __ATOMIC_RELEASE);
        }
        if (live_on) {
            live_flush(&live);
        }
        if (close_file) {
            close_file = 0;
            fclose(out);
            out = NULL;
            char stats_path[320];
            snprintf(stats_path, sizeof stats_path, "%s/last_stats", tap_dir);
            FILE* sf = fopen(stats_path, "w");
            if (sf) {
                fprintf(sf, "frames=%llu dropped=%llu live_send_dropped=%u\n", (unsigned long long)written,
                        (unsigned long long)atomic_load(&dropped), live.send_dropped);
                fclose(sf);
            }
        }
        // polled on purpose: a wake-up from capture() would put a syscall on rl_deploy's 1 kHz real-time path
        usleep((useconds_t)poll_us);
    }
    return NULL;
}

__attribute__((constructor)) static void ecan_tap_init(void) {
    const char* dir = getenv("ECAN_TAP_DIR");
    if (dir && strlen(dir) < sizeof tap_dir) {
        strcpy(tap_dir, dir);
    }
    // the live socket path; empty turns the live stream off, too long for sun_path too
    const char* live = getenv("ECAN_TAP_SOCKET");
    char default_live[sizeof tap_dir + 16];
    snprintf(default_live, sizeof default_live, "%s/live.sock", tap_dir);
    const char* chosen = live ? live : default_live;
    if (strlen(chosen) < sizeof live_path) {
        memcpy(live_path, chosen, strlen(chosen) + 1);
    }
    poll_us = env_long("ECAN_TAP_POLL_US", poll_us, 100, 1000000);
    control_poll_us = env_long("ECAN_TAP_CONTROL_POLL_US", control_poll_us, 1000, 10000000);
    datagram_bytes = (size_t)env_long("ECAN_TAP_DATAGRAM_BYTES", (long)datagram_bytes, 2048, 200000);
    real_socket = dlsym(RTLD_NEXT, "socket");
    extern char* program_invocation_short_name;
    if (strcmp(program_invocation_short_name, "rl_deploy") != 0) {
        return;  // run.sh's bash/sleep/chrt/taskset inherit LD_PRELOAD too; only rl_deploy gets the writer thread
    }
    real_sendto = dlsym(RTLD_NEXT, "sendto");
    real_recvfrom = dlsym(RTLD_NEXT, "recvfrom");
    slots = calloc(SLOT_COUNT, sizeof(Slot));
    if (!slots) {
        return;
    }
    for (int i = 0; i < SLOT_COUNT; i++) {
        atomic_store(&slots[i].ready_seq, 0);
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
