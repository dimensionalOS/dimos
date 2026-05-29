// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Read a pcap of recorded Mid-360 UDP traffic and feed each point/imu
// payload to the existing SDK callbacks. Used by `--replay_pcap` to bypass
// the Livox SDK for deterministic offline regression testing.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <thread>
#include <vector>

#include "livox_lidar_def.h"

namespace pcap_replay {

constexpr uint32_t PCAP_MAGIC_LE_US = 0xa1b2c3d4u;
constexpr uint32_t PCAP_MAGIC_LE_NS = 0xa1b23c4du;
constexpr uint32_t LINKTYPE_ETHERNET = 1u;
constexpr uint16_t ETHERTYPE_IPV4 = 0x0800u;
constexpr uint8_t IPPROTO_UDP = 17u;
constexpr size_t ETH_HDR_LEN = 14;
constexpr size_t IP_MIN_HDR_LEN = 20;
constexpr size_t UDP_HDR_LEN = 8;
constexpr size_t LIVOX_ETH_HDR_LEN = 36;

using PacketCb = std::function<void(LivoxLidarEthernetPacket*)>;
using ClockCb = std::function<void(uint64_t pcap_ts_ns)>;
using IterCb = std::function<void()>;

struct Replayer {
    std::string path;
    uint16_t host_point_port = 0;
    uint16_t host_imu_port = 0;
    PacketCb on_point;
    PacketCb on_imu;
    ClockCb on_clock;
    // Called synchronously after every packet, once the payload has been
    // appended and the virtual clock advanced. The replay path runs the
    // main-loop body here so feeding + processing happen on a single
    // thread — eliminates the feeder-vs-main-loop race on accumulator
    // contents.
    IterCb on_iter;
    std::atomic<bool>* running = nullptr;
    bool realtime = true;
    // Drop Livox packets whose sensor timestamp (pkt->timestamp) is
    // strictly less than this. Used to mimic the SDK warmup window from a
    // paired live run so the algorithm starts from the same first packet
    // in both modes. Comparing on sensor ts (which is identical bit-for-bit
    // between live SDK delivery and pcap replay) is exact; comparing on
    // wall pcap_ts would be off by SDK delivery latency.
    uint64_t skip_until_ns = 0;

    bool run() {
        std::ifstream f(path, std::ios::binary);
        if (!f) {
            fprintf(stderr, "[replay] cannot open %s\n", path.c_str());
            return false;
        }

        uint8_t global_hdr[24];
        f.read(reinterpret_cast<char*>(global_hdr), 24);
        if (!f) {
            fprintf(stderr, "[replay] short read on pcap global header\n");
            return false;
        }
        uint32_t magic;
        std::memcpy(&magic, global_hdr, 4);
        const bool nanos = (magic == PCAP_MAGIC_LE_NS);
        if (magic != PCAP_MAGIC_LE_US && magic != PCAP_MAGIC_LE_NS) {
            fprintf(stderr, "[replay] unsupported pcap magic 0x%08x\n", magic);
            return false;
        }
        uint32_t linktype;
        std::memcpy(&linktype, global_hdr + 20, 4);
        if (linktype != LINKTYPE_ETHERNET) {
            fprintf(stderr, "[replay] unsupported linktype %u (need ETHERNET=1)\n", linktype);
            return false;
        }

        printf("[replay] reading %s (port=%u imu=%u realtime=%d)\n",
               path.c_str(), host_point_port, host_imu_port, realtime ? 1 : 0);

        uint64_t first_pcap_ts_ns = 0;
        std::chrono::steady_clock::time_point start_wall;
        bool seeded = false;

        size_t pkts = 0, pts = 0, imu = 0, other = 0;
        uint8_t rec_hdr[16];
        std::vector<uint8_t> buf;

        while (running == nullptr || running->load()) {
            f.read(reinterpret_cast<char*>(rec_hdr), 16);
            if (!f) {
                break;
            }

            uint32_t ts_sec, ts_sub, incl_len, orig_len;
            std::memcpy(&ts_sec, rec_hdr + 0, 4);
            std::memcpy(&ts_sub, rec_hdr + 4, 4);
            std::memcpy(&incl_len, rec_hdr + 8, 4);
            std::memcpy(&orig_len, rec_hdr + 12, 4);
            (void)orig_len;

            const uint64_t pcap_ts_ns =
                static_cast<uint64_t>(ts_sec) * 1'000'000'000ULL +
                (nanos ? static_cast<uint64_t>(ts_sub) : static_cast<uint64_t>(ts_sub) * 1000ULL);

            buf.resize(incl_len);
            f.read(reinterpret_cast<char*>(buf.data()), incl_len);
            if (!f) {
                break;
            }
            pkts++;

            if (buf.size() < ETH_HDR_LEN) {
                continue;
            }
            uint16_t ethertype = (static_cast<uint16_t>(buf[12]) << 8) | buf[13];
            if (ethertype != ETHERTYPE_IPV4) {
                continue;
            }
            size_t ip_off = ETH_HDR_LEN;
            if (buf.size() < ip_off + IP_MIN_HDR_LEN) {
                continue;
            }
            uint8_t vihl = buf[ip_off];
            if ((vihl >> 4) != 4) {
                continue;
            }
            int ihl = (vihl & 0x0f) * 4;
            if (ihl < static_cast<int>(IP_MIN_HDR_LEN)) {
                continue;
            }
            if (buf[ip_off + 9] != IPPROTO_UDP) {
                continue;
            }
            size_t udp_off = ip_off + ihl;
            if (buf.size() < udp_off + UDP_HDR_LEN) {
                continue;
            }
            uint16_t dst_port = (static_cast<uint16_t>(buf[udp_off + 2]) << 8) | buf[udp_off + 3];
            uint16_t udp_len = (static_cast<uint16_t>(buf[udp_off + 4]) << 8) | buf[udp_off + 5];
            size_t payload_off = udp_off + UDP_HDR_LEN;
            size_t payload_end = std::min(buf.size(), static_cast<size_t>(udp_off + udp_len));
            if (payload_end <= payload_off) {
                continue;
            }
            size_t payload_len = payload_end - payload_off;
            if (payload_len < LIVOX_ETH_HDR_LEN) {
                continue;
            }

            auto* livox_pkt =
                reinterpret_cast<LivoxLidarEthernetPacket*>(buf.data() + payload_off);

            // Sensor-clock skip: drop packets the live SDK wouldn't have
            // seen (those before its first delivered callback) so the
            // algorithm processes the same input set in both modes.
            if (skip_until_ns > 0) {
                uint64_t pkt_ts;
                std::memcpy(&pkt_ts, livox_pkt->timestamp, sizeof(uint64_t));
                if (pkt_ts < skip_until_ns) {
                    continue;
                }
            }

            if (realtime) {
                if (!seeded) {
                    first_pcap_ts_ns = pcap_ts_ns;
                    start_wall = std::chrono::steady_clock::now();
                    seeded = true;
                } else {
                    auto target = start_wall + std::chrono::nanoseconds(pcap_ts_ns - first_pcap_ts_ns);
                    auto now = std::chrono::steady_clock::now();
                    if (target > now) {
                        std::this_thread::sleep_until(target);
                    }
                }
            }

            if (dst_port == host_point_port) {
                if (on_point) {
                    on_point(livox_pkt);
                }
                pts++;
            } else if (dst_port == host_imu_port) {
                if (on_imu) {
                    on_imu(livox_pkt);
                }
                imu++;
            } else {
                other++;
            }
            // Advance the virtual clock AFTER the payload has been added to
            // accumulators. Reverse order would let the main-loop thread see
            // the clock advance and emit a scan that's missing this packet.
            if (on_clock) {
                on_clock(pcap_ts_ns);
            }

            // Run one main-loop iteration synchronously so feeding and
            // processing are strictly serialized in replay mode.
            if (on_iter) {
                on_iter();
            }
        }

        printf("[replay] done: %zu pcap records (point=%zu imu=%zu other=%zu)\n",
               pkts, pts, imu, other);
        return true;
    }
};

}  // namespace pcap_replay
