/*
 * arduino_bridge — Generic serial ↔ LCM relay for DimOS ArduinoModule.
 *
 * This binary is module-agnostic.  It receives topic→LCM channel mappings
 * via CLI args and forwards raw bytes between USB serial and LCM multicast,
 * passing raw payload bytes through (fingerprint is part of the DSP payload).
 *
 * Usage:
 *   ./arduino_bridge \
 *     --serial_port /dev/ttyACM0 \
 *     --baudrate 115200 \
 *     --reconnect true \
 *     --reconnect_interval 2.0 \
 *     --topic_out 1 "/imu#sensor_msgs.Imu" \
 *     --topic_in  2 "/cmd#geometry_msgs.Twist"
 *
 * Copyright 2025-2026 Dimensional Inc.  Apache-2.0.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <signal.h>
#include <string>
#include <thread>
#include <vector>

/* Serial (POSIX) */
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/* LCM */
#include <lcm/lcm-cpp.hpp>

/* DSP protocol constants + CRC */
#include "dsp_protocol.h"

/* ======================================================================
 * Bridge state
 *
 * One Bridge per process.  All state the threads and signal handler touch
 * lives here so the signal handler only needs a single pointer and so the
 * relationship between "user asked us to quit" (`running`) and "serial link
 * is up" (`serial_connected`) is explicit.
 * ====================================================================== */

/* Topic mapping — owned via unique_ptr so that raw pointers stored in the
 * lookup maps (and in RawHandler::tm) are never invalidated by reallocation
 * of the containing vector. */
struct TopicMapping {
    uint16_t topic_id;
    std::string lcm_channel;    /* full "name#msg_type" */
    bool is_output;             /* true = Arduino→Host (publish), false = Host→Arduino (subscribe) */
};

/* Forward decl — RawHandler body is below the Bridge so it can reference it. */
class RawHandler;

struct Bridge {
    /* Shutdown: user hit ^C or sent SIGTERM.  Process-lifetime. */
    std::atomic<bool> running{true};
    /* Serial link currently open and threads may use g_serial_fd.  Cycles
     * per reconnect. */
    std::atomic<bool> serial_connected{false};
    /* Set by `lcm_handler_thread` when `handleTimeout` returns an error
     * before clearing `running`.  The main loop reads this after its
     * join()s so we can exit(1) — letting the coordinator restart the
     * bridge — instead of reporting a clean shutdown. */
    std::atomic<bool> lcm_failed{false};

    int serial_fd{-1};
    std::mutex serial_write_mutex;

    std::vector<std::unique_ptr<TopicMapping>> topics;

    lcm::LCM *lcm{nullptr};
    std::map<std::string, int64_t> hash_registry;

    std::map<uint16_t, TopicMapping *> topic_out_map;
    std::map<std::string, TopicMapping *> topic_in_map;
    std::vector<std::unique_ptr<RawHandler>> raw_handlers;

    std::string serial_port;
    int baudrate{115200};
    bool reconnect{true};
    float reconnect_interval{2.0f};
};

/* Single process-global pointer the signal handler touches.  All other
 * code takes the Bridge by reference. */
static Bridge *g_bridge = nullptr;

/* ======================================================================
 * CLI Parsing
 * ====================================================================== */

/* Returns the termios speed constant for `baud`, or B0 with *ok=false on
 * unsupported rates.
 *
 * Darwin's <termios.h> only defines POSIX-standard constants up to
 * B230400; B460800 / B500000 / B576000 / B921600 / B1000000 are
 * Linux-specific extensions.  We gate them behind `__linux__` so the
 * bridge builds cleanly on macOS — useful for the virtual-Arduino
 * path where the bridge talks to a QEMU PTY and any baud rate is
 * effectively a no-op anyway. */
static speed_t baud_to_speed(int baud, bool *ok)
{
    *ok = true;
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
#ifdef __linux__
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 576000:  return B576000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
#endif
        default:
            *ok = false;
            return B0;
    }
}

/* `exit(1)` on unknown flags, missing required arg values, or unsupported
 * baud rates.  Silently falling back to a default is a footgun. */
static void parse_args(Bridge &b, int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);

        if (arg == "--serial_port" && i + 1 < argc) {
            b.serial_port = argv[++i];
        } else if (arg == "--baudrate" && i + 1 < argc) {
            b.baudrate = std::atoi(argv[++i]);
            bool ok;
            (void)baud_to_speed(b.baudrate, &ok);
            if (!ok) {
                fprintf(stderr, "[bridge] Unsupported baud rate: %d\n", b.baudrate);
                exit(1);
            }
        } else if (arg == "--reconnect" && i + 1 < argc) {
            std::string val(argv[++i]);
            b.reconnect = (val == "true" || val == "1");
        } else if (arg == "--reconnect_interval" && i + 1 < argc) {
            b.reconnect_interval = std::atof(argv[++i]);
        } else if ((arg == "--topic_out" || arg == "--topic_in") && i + 2 < argc) {
            auto tm = std::make_unique<TopicMapping>();
            tm->topic_id = (uint16_t)std::atoi(argv[++i]);
            tm->lcm_channel = argv[++i];
            tm->is_output = (arg == "--topic_out");
            b.topics.push_back(std::move(tm));
        } else {
            fprintf(stderr, "[bridge] Unknown or malformed argument: %s\n", arg.c_str());
            exit(1);
        }
    }
}

/* ======================================================================
 * LCM Fingerprint Hash Registry
 *
 * The 2-byte topic DSP protocol includes the 8-byte LCM fingerprint
 * INSIDE the DSP payload, so the bridge is a pure passthrough — no
 * prepending or stripping needed.  We keep the hash registry for
 * validation and logging purposes.
 * ====================================================================== */

/*
 * Include all LCM C++ message headers we support.
 * The fingerprint hash is available via Type::getHash().
 */
#include "std_msgs/Time.hpp"
#include "std_msgs/Bool.hpp"
#include "std_msgs/Int32.hpp"
#include "std_msgs/Float32.hpp"
#include "std_msgs/Float64.hpp"
#include "std_msgs/ColorRGBA.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "geometry_msgs/Point.hpp"
#include "geometry_msgs/Point32.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Pose.hpp"
#include "geometry_msgs/Pose2D.hpp"
#include "geometry_msgs/Twist.hpp"
#include "geometry_msgs/Accel.hpp"
#include "geometry_msgs/Transform.hpp"
#include "geometry_msgs/Wrench.hpp"
#include "geometry_msgs/Inertia.hpp"
#include "geometry_msgs/PoseWithCovariance.hpp"
#include "geometry_msgs/TwistWithCovariance.hpp"
#include "geometry_msgs/AccelWithCovariance.hpp"

static void init_hash_registry(Bridge &b)
{
    /* Register all known types.
     *
     * NOTE: this list is kept in sync with three other places and there is
     * a Python test (`test_arduino_msg_registry_sync`) that fails CI if any
     * of them drift:
     *   - dimos/core/arduino_module.py :: _KNOWN_TYPE_HEADERS
     *   - dimos/hardware/arduino/common/arduino_msgs (Arduino-side .h)
     *   - this function (C++ bridge hash registry)
     */
    b.hash_registry["std_msgs.Time"]       = std_msgs::Time::getHash();
    b.hash_registry["std_msgs.Bool"]       = std_msgs::Bool::getHash();
    b.hash_registry["std_msgs.Int32"]      = std_msgs::Int32::getHash();
    b.hash_registry["std_msgs.Float32"]    = std_msgs::Float32::getHash();
    b.hash_registry["std_msgs.Float64"]    = std_msgs::Float64::getHash();
    b.hash_registry["std_msgs.ColorRGBA"]  = std_msgs::ColorRGBA::getHash();

    b.hash_registry["geometry_msgs.Vector3"]    = geometry_msgs::Vector3::getHash();
    b.hash_registry["geometry_msgs.Point"]      = geometry_msgs::Point::getHash();
    b.hash_registry["geometry_msgs.Point32"]    = geometry_msgs::Point32::getHash();
    b.hash_registry["geometry_msgs.Quaternion"] = geometry_msgs::Quaternion::getHash();
    b.hash_registry["geometry_msgs.Pose"]       = geometry_msgs::Pose::getHash();
    b.hash_registry["geometry_msgs.Pose2D"]     = geometry_msgs::Pose2D::getHash();
    b.hash_registry["geometry_msgs.Twist"]      = geometry_msgs::Twist::getHash();
    b.hash_registry["geometry_msgs.Accel"]      = geometry_msgs::Accel::getHash();
    b.hash_registry["geometry_msgs.Transform"]  = geometry_msgs::Transform::getHash();
    b.hash_registry["geometry_msgs.Wrench"]     = geometry_msgs::Wrench::getHash();
    b.hash_registry["geometry_msgs.Inertia"]    = geometry_msgs::Inertia::getHash();
    b.hash_registry["geometry_msgs.PoseWithCovariance"]  = geometry_msgs::PoseWithCovariance::getHash();
    b.hash_registry["geometry_msgs.TwistWithCovariance"] = geometry_msgs::TwistWithCovariance::getHash();
    b.hash_registry["geometry_msgs.AccelWithCovariance"] = geometry_msgs::AccelWithCovariance::getHash();
}

/* Extract "msg_type" from "topic_name#msg_type" */
static std::string extract_msg_type(const std::string &channel)
{
    auto pos = channel.find('#');
    if (pos == std::string::npos) return "";
    return channel.substr(pos + 1);
}

/* Extract "topic_name" from "topic_name#msg_type" */
static std::string extract_topic_name(const std::string &channel)
{
    auto pos = channel.find('#');
    if (pos == std::string::npos) return channel;
    return channel.substr(0, pos);
}

/* Validate that all topic message types are in the hash registry (for logging). */
static bool validate_topic_types(Bridge &b)
{
    for (auto &tm : b.topics) {
        std::string msg_type = extract_msg_type(tm->lcm_channel);
        auto it = b.hash_registry.find(msg_type);
        if (it == b.hash_registry.end()) {
            fprintf(stderr,
                    "[bridge] Unknown message type: %s (topic_id=%u, channel=%s)\n",
                    msg_type.c_str(), tm->topic_id, tm->lcm_channel.c_str());
            return false;
        }
    }
    return true;
}

/* ======================================================================
 * Serial Port
 * ====================================================================== */

static int serial_open(const std::string &port, int baud)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[bridge] Cannot open %s: %s\n", port.c_str(), strerror(errno));
        return -1;
    }

    /* Clear O_NONBLOCK after open (we want blocking reads in the reader thread) */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tcgetattr(fd, &tio);

    /* Raw mode: no echo, no canonical, no signals */
    cfmakeraw(&tio);

    /* 8N1 */
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;

    /* No flow control */
    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* Set baud — parse_args already validated this, so `ok` should always be
     * true here.  Assert to be safe. */
    bool speed_ok;
    speed_t speed = baud_to_speed(baud, &speed_ok);
    if (!speed_ok) {
        fprintf(stderr, "[bridge] BUG: serial_open called with unsupported baud %d\n", baud);
        close(fd);
        return -1;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    /* Read timeout: return after 100ms or 1 byte, whichever first */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;  /* 100ms in deciseconds */

    tcsetattr(fd, TCSANOW, &tio);
    tcflush(fd, TCIOFLUSH);

    return fd;
}

static void serial_close(int fd)
{
    if (fd >= 0) close(fd);
}

/* ======================================================================
 * Serial → LCM (reader thread)
 * ====================================================================== */

static void serial_reader_thread(Bridge &b)
{
    /* Framing state lives entirely inside `dsp_parser` — shared with the
     * AVR side via `dsp_protocol.h`.  One implementation, one place to
     * change when the wire format evolves.  Reads bytes in small chunks
     * to amortize the `read()` syscall (VMIN=0/VTIME=1 already gives us
     * a bounded blocking read, so bulk reads are safe). */
    struct dsp_parser parser;
    dsp_parser_init(&parser);

    uint8_t chunk[64];

    /* Exit on either global shutdown or a serial disconnect flagged by the
     * writer path.  `serial_connected` being a separate atomic means that
     * `signal_handler` flipping `running` to false and the writer flipping
     * `serial_connected` to false don't race each other's meaning. */
    while (b.running.load() && b.serial_connected.load()) {
        int n = read(b.serial_fd, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[bridge] Serial read error: %s\n", strerror(errno));
            b.serial_connected.store(false);
            break;
        }
        if (n == 0) continue;  /* VTIME timeout, loop back */

        for (int i = 0; i < n; i++) {
            enum dsp_parse_event ev = dsp_feed_byte(&parser, chunk[i]);

            if (ev == DSP_PARSE_CRC_FAIL) {
                fprintf(stderr, "[bridge] CRC mismatch on topic %u\n", parser.rx_topic);
                continue;
            }
            if (ev == DSP_PARSE_OVERFLOW) {
                fprintf(stderr, "[bridge] Frame length %u exceeds DSP_MAX_PAYLOAD=%d on topic %u\n",
                        parser.rx_len, DSP_MAX_PAYLOAD, parser.rx_topic);
                continue;
            }
            if (ev != DSP_PARSE_MESSAGE) continue;

            /* Handle frame */
            if (parser.rx_topic == DSP_TOPIC_DEBUG) {
                /* Debug: print to stdout */
                fwrite(parser.rx_buf, 1, parser.rx_len, stdout);
                fflush(stdout);
            } else {
                /* Data: payload already contains [8B fingerprint][data],
                 * publish directly to LCM as a pure passthrough. */
                auto it = b.topic_out_map.find(parser.rx_topic);
                if (it != b.topic_out_map.end()) {
                    TopicMapping *tm = it->second;
                    b.lcm->publish(tm->lcm_channel, parser.rx_buf, parser.rx_len);
                } else {
                    fprintf(stderr, "[bridge] Unknown outbound topic: %u\n", parser.rx_topic);
                }
            }
        }
    }
}

/* ======================================================================
 * LCM → Serial (subscription handler)
 * ====================================================================== */

/* write_all — loop until `len` bytes are written or a hard error occurs.
 *
 * On EINTR we retry; on any other error we return false so the caller can
 * flag the serial link down and let the reconnect loop run.  A partial
 * write on a dying USB device would otherwise corrupt the DSP frame. */
static bool write_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = static_cast<const uint8_t *>(buf);
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t n = ::write(fd, p, remaining);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;  /* shouldn't happen on a blocking fd */
        p += (size_t)n;
        remaining -= (size_t)n;
    }
    return true;
}

/* Forward declaration */
static void send_lcm_to_serial(Bridge &b,
                               const lcm::ReceiveBuffer *rbuf,
                               TopicMapping *tm);

class RawHandler {
public:
    Bridge *bridge;
    TopicMapping *tm;
    RawHandler(Bridge *br, TopicMapping *t) : bridge(br), tm(t) {}
    void handle(const lcm::ReceiveBuffer *rbuf, const std::string & /*channel*/) {
        send_lcm_to_serial(*bridge, rbuf, tm);
    }
};

static void send_lcm_to_serial(Bridge &b,
                               const lcm::ReceiveBuffer *rbuf,
                               TopicMapping *tm)
{
    /* LCM message already contains [8B fingerprint][data].  Pass the
     * FULL payload (with fingerprint) into the DSP frame — pure passthrough. */
    size_t data_size = (size_t)rbuf->data_size;
    const uint8_t *payload = (const uint8_t *)rbuf->data;
    size_t payload_len_raw = data_size;

    if (payload_len_raw > DSP_MAX_PAYLOAD) {
        fprintf(stderr,
                "[bridge] Dropping LCM message on %s: payload %zu > DSP_MAX_PAYLOAD %d\n",
                tm->lcm_channel.c_str(), payload_len_raw, DSP_MAX_PAYLOAD);
        return;
    }
    uint16_t payload_len = (uint16_t)payload_len_raw;

    /* Build DSP frame header: START + TOPIC(2B LE) + LENGTH(2B LE) */
    uint8_t header[DSP_HEADER_SIZE];
    header[0] = DSP_START_BYTE;
    header[1] = (uint8_t)(tm->topic_id & 0xFF);
    header[2] = (uint8_t)((tm->topic_id >> 8) & 0xFF);
    header[3] = (uint8_t)(payload_len & 0xFF);
    header[4] = (uint8_t)((payload_len >> 8) & 0xFF);

    /* CRC-8/MAXIM over TOPIC_LO + TOPIC_HI + LEN_LO + LEN_HI + PAYLOAD, incremental. */
    uint8_t crc = 0x00;
    crc = _dsp_crc8_table[crc ^ header[1]];
    crc = _dsp_crc8_table[crc ^ header[2]];
    crc = _dsp_crc8_table[crc ^ header[3]];
    crc = _dsp_crc8_table[crc ^ header[4]];
    for (uint16_t k = 0; k < payload_len; k++) {
        crc = _dsp_crc8_table[crc ^ payload[k]];
    }

    /* Write to serial (thread-safe w.r.t. other writers).
     *
     * If any write fails (USB disconnect, short write on a dead fd, etc.)
     * we flag `serial_connected` false so the reader thread bails and the
     * reconnect loop takes over.  Dropping a partial frame is strictly
     * better than continuing to corrupt the outbound stream. */
    std::lock_guard<std::mutex> lock(b.serial_write_mutex);
    if (!b.serial_connected.load()) return;
    bool ok = write_all(b.serial_fd, header, DSP_HEADER_SIZE);
    if (ok && payload_len > 0) {
        ok = write_all(b.serial_fd, payload, payload_len);
    }
    if (ok) {
        ok = write_all(b.serial_fd, &crc, 1);
    }
    if (!ok) {
        fprintf(stderr,
                "[bridge] Serial write failed on topic %u (%s): %s — flagging disconnect\n",
                tm->topic_id, tm->lcm_channel.c_str(), strerror(errno));
        b.serial_connected.store(false);
    }
}

static void lcm_handler_thread(Bridge &b)
{
    while (b.running.load() && b.serial_connected.load()) {
        int ret = b.lcm->handleTimeout(100);  /* 100ms timeout */
        if (ret < 0) {
            /* LCM is sick (multicast group went away, kernel socket
             * broke, etc.) — cycling the serial port would not help and
             * would unnecessarily discard in-flight data on a link that
             * is still perfectly healthy.  Instead, clear `running` so
             * the main loop exits with a non-zero status and the
             * coordinator can restart the whole bridge subprocess on a
             * fresh LCM instance. */
            fprintf(stderr, "[bridge] LCM handle error — exiting so coordinator can restart\n");
            b.lcm_failed.store(true);
            b.running.store(false);
            break;
        }
    }
}

/* ======================================================================
 * Signal handling
 * ====================================================================== */

static void signal_handler(int /*sig*/)
{
    if (g_bridge) g_bridge->running.store(false);
}

/* Sleep for at most `seconds`, waking early if `running` is cleared. */
static void interruptible_sleep(Bridge &b, float seconds)
{
    const int step_ms = 50;
    const int total_ms = (int)(seconds * 1000.0f);
    int elapsed = 0;
    while (elapsed < total_ms && b.running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
        elapsed += step_ms;
    }
}

/* ======================================================================
 * Main
 * ====================================================================== */

int main(int argc, char **argv)
{
    Bridge bridge;
    g_bridge = &bridge;

    parse_args(bridge, argc, argv);

    if (bridge.serial_port.empty()) {
        fprintf(stderr, "Usage: arduino_bridge --serial_port <port> --baudrate <baud> "
                        "[--topic_out <id> <channel>] [--topic_in <id> <channel>] ...\n");
        return 1;
    }

    /* Initialize hash registry and validate topic message types */
    init_hash_registry(bridge);
    if (!validate_topic_types(bridge)) {
        return 1;
    }

    /* Build lookup maps — use the unique_ptr-owned storage so raw pointers
     * into the vector remain valid. */
    for (auto &tm : bridge.topics) {
        if (tm->is_output) {
            bridge.topic_out_map[tm->topic_id] = tm.get();
        } else {
            bridge.topic_in_map[tm->lcm_channel] = tm.get();
        }
    }

    /* Signal handlers */
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    /* Init LCM */
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[bridge] LCM init failed\n");
        return 1;
    }
    bridge.lcm = &lcm;

    /* Subscribe to inbound LCM topics */
    for (auto &tm : bridge.topics) {
        if (!tm->is_output) {
            auto handler = std::make_unique<RawHandler>(&bridge, tm.get());
            lcm.subscribe(tm->lcm_channel, &RawHandler::handle, handler.get());
            bridge.raw_handlers.push_back(std::move(handler));
            printf("[bridge] Subscribed LCM→Serial: topic %u ← %s\n",
                   tm->topic_id, tm->lcm_channel.c_str());
        } else {
            printf("[bridge] Serial→LCM: topic %u → %s\n",
                   tm->topic_id, tm->lcm_channel.c_str());
        }
    }

    /* Open serial port */
    printf("[bridge] Opening %s at %d baud\n", bridge.serial_port.c_str(), bridge.baudrate);

    while (bridge.running.load()) {
        bridge.serial_fd = serial_open(bridge.serial_port, bridge.baudrate);
        if (bridge.serial_fd < 0) {
            if (!bridge.reconnect) return 1;
            fprintf(stderr, "[bridge] Retrying in %.1fs...\n", bridge.reconnect_interval);
            interruptible_sleep(bridge, bridge.reconnect_interval);
            continue;
        }

        printf("[bridge] Serial port opened (fd=%d)\n", bridge.serial_fd);
        bridge.serial_connected.store(true);

        /* Start threads */
        std::thread reader([&bridge] { serial_reader_thread(bridge); });
        std::thread lcm_thread([&bridge] { lcm_handler_thread(bridge); });

        /* Wait for reader to exit (serial disconnect or shutdown) */
        reader.join();

        /* Reader bailed — ensure connectivity flag is false and join LCM. */
        bridge.serial_connected.store(false);
        lcm_thread.join();

        serial_close(bridge.serial_fd);
        bridge.serial_fd = -1;

        if (!bridge.reconnect || !bridge.running.load()) break;

        /* Reconnect.  DO NOT touch `running` here — only the signal handler
         * clears it, and we don't want to overwrite a ^C that arrives during
         * the backoff sleep. */
        printf("[bridge] Disconnected, reconnecting in %.1fs...\n", bridge.reconnect_interval);
        interruptible_sleep(bridge, bridge.reconnect_interval);
    }

    printf("[bridge] Shutting down\n");
    /* Distinguish graceful shutdown (SIGTERM/SIGINT) from an LCM failure
     * that forced us out of the main loop.  Non-zero exit tells the
     * coordinator to restart us with a fresh LCM subscriber. */
    return bridge.lcm_failed.load() ? 1 : 0;
}
