// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Golden harness: replays an L0 .plio file through the Point-LIO core with a
// fixed feed sequence (no wall clock, no threads) and dumps the outputs the
// Rust port is checked against. Format: ../rust/REPLAY.md.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/make_shared.hpp>
#include <nlohmann/json.hpp>

// laserMapping.hpp defines the core's globals, so this TU sees them directly.
#include "pointlio.hpp"
#include "pointlio_debug.hpp"

#include "params.hpp"

namespace {

constexpr double kGravity = 9.80665;  // GRAVITY_MS2 in the livox crate

struct Reader {
    FILE* f;
    template <class T> T rd() {
        T v;
        if (std::fread(&v, sizeof v, 1, f) != 1) throw std::runtime_error("truncated .plio");
        return v;
    }
    bool eof() {
        int c = std::getc(f);
        if (c == EOF) return true;
        std::ungetc(c, f);
        return false;
    }
};

struct Writer {
    FILE* f;
    template <class T> void wr(const T& v) { std::fwrite(&v, sizeof v, 1, f); }
};

void feed_imu(LaserMapping& lm, Reader& in) {
    auto msg = boost::make_shared<custom_messages::Imu>();
    uint64_t ts_ns = in.rd<uint64_t>();
    double g[3], a[3];
    for (double& v : g) v = in.rd<double>();
    for (double& v : a) v = in.rd<double>();
    msg->header.stamp = custom_messages::Time().fromSec(static_cast<double>(ts_ns) / 1e9);
    msg->header.seq = 0;
    msg->header.frame_id = "livox_frame";
    msg->orientation.x = msg->orientation.y = msg->orientation.z = 0.0;
    msg->orientation.w = 1.0;
    for (int j = 0; j < 9; ++j) {
        msg->orientation_covariance[j] = 0.0;
        msg->angular_velocity_covariance[j] = 0.0;
        msg->linear_acceleration_covariance[j] = 0.0;
    }
    msg->angular_velocity.x = g[0];
    msg->angular_velocity.y = g[1];
    msg->angular_velocity.z = g[2];
    msg->linear_acceleration.x = a[0] / kGravity;
    msg->linear_acceleration.y = a[1] / kGravity;
    msg->linear_acceleration.z = a[2] / kGravity;
    lm.imu_cbk(msg);
}

void feed_lidar(LaserMapping& lm, Reader& in) {
    auto msg = boost::make_shared<custom_messages::CustomMsg>();
    uint64_t start_ns = in.rd<uint64_t>();
    uint32_t n = in.rd<uint32_t>();
    msg->header.seq = 0;
    msg->header.stamp = custom_messages::Time().fromSec(static_cast<double>(start_ns) / 1e9);
    msg->header.frame_id = "livox_frame";
    msg->timebase = start_ns;
    msg->lidar_id = 0;
    for (auto& r : msg->rsvd) r = 0;
    msg->point_num = n;
    msg->points.resize(n);
    for (auto& cp : msg->points) {
        cp.x = in.rd<float>();
        cp.y = in.rd<float>();
        cp.z = in.rd<float>();
        // Driver publishes reflectivity / 255 as f32; this inverts it exactly.
        cp.reflectivity = static_cast<usi>(std::lround(in.rd<float>() * 255.0f));
        cp.offset_time = in.rd<uint32_t>();
        cp.tag = in.rd<uint8_t>();
        cp.line = 0;
    }
    lm.livox_pcl_cbk(msg);
}

// One frame of intermediates, read straight off the core's globals.
void dump_frame(Writer& out, uint32_t idx, double ts) {
    const auto& pts = feats_down_body->points;
    out.wr(idx);
    out.wr(ts);
    out.wr(static_cast<uint32_t>(feats_down_size));
    for (int i = 0; i < feats_down_size; ++i) {
        out.wr(pts[i].x); out.wr(pts[i].y); out.wr(pts[i].z);
    }
    out.wr(static_cast<uint32_t>(effct_feat_num));
    const auto& x = kf_output.x_;
    auto wr3 = [&](const V3D& v) { out.wr(v(0)); out.wr(v(1)); out.wr(v(2)); };
    auto wr9 = [&](const M3D& m) { for (int i = 0; i < 9; ++i) out.wr(m.data()[i]); };
    wr3(x.pos); wr9(x.rot); wr9(x.offset_R_L_I); wr3(x.offset_T_L_I);
    wr3(x.vel); wr3(x.omg); wr3(x.acc); wr3(x.gravity); wr3(x.bg); wr3(x.ba);
    for (int r = 0; r < 30; ++r)
        for (int c = 0; c < 30; ++c) out.wr(kf_output.P_(r, c));
    for (int i = 0; i < feats_down_size; ++i) {
        const PointVector& nbr = Nearest_Points[i];
        uint8_t selected = point_selected_surf[i] ? 1 : 0;
        out.wr(selected);
        out.wr(static_cast<uint8_t>(nbr.size()));
        for (const auto& p : nbr) { out.wr(p.x); out.wr(p.y); out.wr(p.z); }
        // The core keeps only the last group's planes (normvec is per-group), so
        // recompute from the retained neighbours: same call, same inputs.
        VF(4) plane = VF(4)::Zero();
        if (selected && nbr.size() >= NUM_MATCH_POINTS) esti_plane(plane, nbr, plane_thr);
        for (int j = 0; j < 4; ++j) out.wr(plane(j));
    }
}

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, static_cast<size_t>(p * v.size()))];
}

}  // namespace

int main(int argc, char** argv) {
    std::string replay, config, outdir;
    uint32_t frames = 30;
    bool stats = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&] { if (++i >= argc) throw std::runtime_error(a + " needs a value"); return std::string(argv[i]); };
        if (a == "--replay") replay = next();
        else if (a == "--config") config = next();
        else if (a == "--out") outdir = next();
        else if (a == "--frames") frames = std::stoul(next());
        else if (a == "--stats") stats = true;
        else throw std::runtime_error("unknown arg " + a);
    }
    if (replay.empty() || config.empty() || outdir.empty()) {
        std::fprintf(stderr, "usage: pointlio_harness --replay f.plio --config c.json --out dir [--frames K] [--stats]\n");
        return 2;
    }

    std::ifstream cfg_in(config);
    if (!cfg_in) throw std::runtime_error("cannot open " + config);
    PointLioConfig cfg = dimos::native::Config(nlohmann::json::parse(cfg_in)).parse<PointLioConfig>();
    pointlio_debug = cfg.debug;
    LaserMapping lm(to_params(cfg), cfg.msr_freq, cfg.main_freq);

    std::filesystem::create_directories(outdir);
    std::filesystem::copy_file(config, outdir + "/config.json",
                               std::filesystem::copy_options::overwrite_existing);
    FILE* fin = std::fopen(replay.c_str(), "rb");
    if (!fin) throw std::runtime_error("cannot open " + replay);
    Reader in{fin};
    char magic[4];
    if (std::fread(magic, 1, 4, fin) != 4 || std::memcmp(magic, "PLIO", 4) != 0)
        throw std::runtime_error("not a .plio file");
    if (in.rd<uint32_t>() != 1) throw std::runtime_error("unsupported .plio version");
    in.rd<double>();  // frame_hz, informational

    FILE* ftum = std::fopen((outdir + "/trajectory.tum").c_str(), "w");
    Writer fbin{std::fopen((outdir + "/frames.bin").c_str(), "wb")};
    if (!ftum || !fbin.f) throw std::runtime_error("cannot write to " + outdir);

    custom_messages::Odometry odom;
    uint32_t processed = 0;
    std::vector<double> lidar_ms;
    auto t_start = std::chrono::steady_clock::now();
    while (!in.eof()) {
        uint8_t kind = in.rd<uint8_t>();
        if (kind == 0) { feed_imu(lm, in); continue; }
        if (kind != 1) throw std::runtime_error("bad record kind");
        auto t0 = std::chrono::steady_clock::now();
        feed_lidar(lm, in);
        // Drain: run_once returns false both on "no frame" and on the map-init
        // frame, so stop only when it consumed nothing.
        for (;;) {
            size_t before = lidar_buffer.size();
            if (lm.run_once(odom)) {
                double ts = odom.header.stamp.toSec();
                const auto& p = odom.pose.pose;
                std::fprintf(ftum, "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", ts,
                             p.position.x, p.position.y, p.position.z, p.orientation.x,
                             p.orientation.y, p.orientation.z, p.orientation.w);
                if (processed < frames) dump_frame(fbin, processed, ts);
                ++processed;
            }
            if (lidar_buffer.size() == before) break;
        }
        lidar_ms.push_back(std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - t0).count());
    }
    double total_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    std::fclose(ftum);
    std::fclose(fbin.f);
    std::fclose(fin);

    std::fprintf(stderr, "frames processed: %u (dumped %u)\n", processed, std::min(processed, frames));
    if (stats) {
        std::fprintf(stderr, "lidar records: %zu  per-record ms p50 %.3f p99 %.3f max %.3f  total %.3f s\n",
                     lidar_ms.size(), percentile(lidar_ms, 0.5), percentile(lidar_ms, 0.99),
                     lidar_ms.empty() ? 0.0 : *std::max_element(lidar_ms.begin(), lidar_ms.end()),
                     total_s);
    }
    return 0;
}
