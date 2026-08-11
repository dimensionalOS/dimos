// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Offline replay for CuvslamOdometry: the real module, no transport, no pacing.
//
// A replay log (export_replay_log.py) holds the exact LCM payloads the live
// topics would carry, in arrival order. Each record is pushed through the same
// decode dispatch the wire uses and the handler queue is drained before the
// next record, so behaviour matches the real-time replay message for message —
// the only thing missing is the wait between frames.
//
//   cuvslam_replay --log frames.cvrlog --config config.json --out poses.json \
//       [--override '{"speed_gate_max_linear": 0.0}']
//
// `config.json` is the module's full config blob (CuvslamConfig.to_config_dict()
// from python, which owns every default). Outputs are TrajectorySink-format
// rows: [ts, x, y, z, qx, qy, qz, qw, translation_std]; the corrected stream
// lands next to --out with a `_corrected` suffix.

#define CUVSLAM_REPLAY_HARNESS
#include "cuvslam_main.cpp"

#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {

constexpr char LOG_MAGIC[8] = {'C', 'V', 'R', 'L', 'O', 'G', '0', '1'};

/// Collects what the module publishes instead of putting it on a wire.
class CaptureTransport : public dimos::native::Transport {
public:
    void publish(const std::string& channel, std::vector<uint8_t> data) override {
        if (channel.find("odometry") == std::string::npos) {
            return;  // tf carries the same poses; the sinks score odometry only
        }
        const nav_msgs::Odometry msg =
            dimos::native::lcm_decode<nav_msgs::Odometry>(data.data(), data.size());
        const double translation_std = std::sqrt(std::max(
            {msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14]}));
        const std::array<double, 9> row{
            static_cast<double>(msg.header.stamp.sec) + msg.header.stamp.nsec / 1.0e9,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            translation_std};
        std::lock_guard<std::mutex> lock(mutex_);
        if (channel.find("corrected") != std::string::npos) {
            corrected_.push_back(row);
        } else {
            poses_.push_back(row);
        }
    }

    void subscribe(const std::string&, dimos::native::Dispatch) override {}

    void write(const std::string& out_path) {
        std::lock_guard<std::mutex> lock(mutex_);
        write_rows(out_path, poses_);
        std::string corrected_path = out_path;
        const std::size_t dot = corrected_path.rfind(".json");
        corrected_path.insert(dot == std::string::npos ? corrected_path.size() : dot,
                              "_corrected");
        write_rows(corrected_path, corrected_);
        std::printf("[replay] wrote %zu poses -> %s\n", poses_.size(), out_path.c_str());
        std::printf("[replay] wrote %zu corrected -> %s\n", corrected_.size(),
                    corrected_path.c_str());
    }

private:
    static void write_rows(const std::string& path,
                           const std::vector<std::array<double, 9>>& rows) {
        std::ofstream out(path);
        out << std::setprecision(17) << "[";
        for (std::size_t i = 0; i < rows.size(); ++i) {
            out << (i ? ", " : "") << "[";
            for (std::size_t j = 0; j < rows[i].size(); ++j) {
                out << (j ? ", " : "");
                // Python's json spells non-finite NaN/Infinity; C++ streams do not.
                if (std::isfinite(rows[i][j])) {
                    out << rows[i][j];
                } else if (std::isnan(rows[i][j])) {
                    out << "NaN";
                } else {
                    out << (rows[i][j] > 0 ? "Infinity" : "-Infinity");
                }
            }
            out << "]";
        }
        out << "]";
    }

    std::mutex mutex_;
    std::vector<std::array<double, 9>> poses_;
    std::vector<std::array<double, 9>> corrected_;
};

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

struct Args {
    std::string log_path;
    std::string config_path;
    std::string out_path;
    std::string override_json;
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (i + 1 >= argc) {
            throw std::runtime_error("flag needs a value: " + flag);
        }
        const std::string value = argv[++i];
        if (flag == "--log") {
            args.log_path = value;
        } else if (flag == "--config") {
            args.config_path = value;
        } else if (flag == "--out") {
            args.out_path = value;
        } else if (flag == "--override") {
            args.override_json = value;
        } else {
            throw std::runtime_error("unknown flag: " + flag);
        }
    }
    if (args.log_path.empty() || args.config_path.empty() || args.out_path.empty()) {
        throw std::runtime_error(
            "usage: cuvslam_replay --log FILE --config FILE --out FILE "
            "[--override JSON]");
    }
    return args;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);

        nlohmann::json config_blob = nlohmann::json::parse(read_file(args.config_path));
        if (!args.override_json.empty()) {
            const nlohmann::json overrides = nlohmann::json::parse(args.override_json);
            for (const auto& [key, value] : overrides.items()) {
                config_blob[key] = value;
            }
        }

        // Ports map straight onto themselves: with no wire there is nothing to route.
        std::unordered_map<std::string, std::string> topics;
        for (const char* port : {"tf", "camera_info", "image", "depth_camera_info",
                                 "depth_image", "imu", "odometry", "corrected_odometry"}) {
            topics.emplace(port, port);
        }

        dimos::native::Notifier notifier;
        Builder builder(std::move(topics), &notifier);
        CuvslamOdometry module;
        Config config{std::move(config_blob)};
        module.build(builder, config);
        config.enforce_all_consumed();

        auto capture = std::make_unique<CaptureTransport>();
        std::vector<std::thread> workers;
        // Joins on every exit path; a joinable thread destroyed mid-unwind terminates.
        struct JoinWorkers {
            Builder& builder;
            std::vector<std::thread>& workers;
            ~JoinWorkers() {
                for (const auto& queue : builder.publish_queues()) {
                    queue->stop();
                }
                for (std::thread& worker : workers) {
                    if (worker.joinable()) {
                        worker.join();
                    }
                }
            }
        } join_workers{builder, workers};
        workers.reserve(builder.publish_queues().size());
        for (const auto& queue : builder.publish_queues()) {
            workers.emplace_back(dimos::native::publish_worker_loop, queue.get(),
                                 capture.get());
        }
        std::unordered_map<std::string, dimos::native::Dispatch> routes;
        for (const auto& route : builder.routes()) {
            routes.emplace(route.first, route.second);
        }

        module.setup();

        std::ifstream log(args.log_path, std::ios::binary);
        char magic[sizeof(LOG_MAGIC)];
        if (!log.read(magic, sizeof(magic)) ||
            std::memcmp(magic, LOG_MAGIC, sizeof(LOG_MAGIC)) != 0) {
            throw std::runtime_error("not a CVRLOG01 file: " + args.log_path);
        }
        const auto started = std::chrono::steady_clock::now();
        std::uint64_t records = 0;
        std::vector<std::uint8_t> payload;
        while (true) {
            std::uint32_t topic_len = 0;
            if (!log.read(reinterpret_cast<char*>(&topic_len), sizeof(topic_len))) {
                break;
            }
            if (topic_len > 256) {
                throw std::runtime_error("desynced log: topic_len " +
                                         std::to_string(topic_len) + " at record " +
                                         std::to_string(records));
            }
            std::string topic(topic_len, '\0');
            log.read(topic.data(), topic_len);
            std::uint64_t data_len = 0;
            log.read(reinterpret_cast<char*>(&data_len), sizeof(data_len));
            if (data_len > (100ULL << 20)) {
                throw std::runtime_error("desynced log: data_len " +
                                         std::to_string(data_len) + " at record " +
                                         std::to_string(records));
            }
            payload.resize(data_len);
            log.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(data_len));
            if (!log) {
                throw std::runtime_error("truncated record in " + args.log_path);
            }
            const std::uint8_t* data = payload.data();
            ++records;

            const auto route = routes.find(topic);
            if (route == routes.end()) {
                continue;  // a stream this camera_mode has no input for
            }
            route->second(data, data_len);
            // Drain before the next record: queue depth stays at one, so handler
            // order is exactly arrival order, as on the live dispatch loop.
            for (dimos::native::InputPort* port : builder.input_ports()) {
                while (port->drain_one()) {
                }
            }
        }
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
                .count();
        std::printf("[replay] %llu records in %.1f s\n",
                    static_cast<unsigned long long>(records), elapsed);

        module.teardown();
        for (const auto& queue : builder.publish_queues()) {
            queue->stop();
        }
        for (std::thread& worker : workers) {
            worker.join();
        }
        capture->write(args.out_path);
        std::fflush(stdout);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "cuvslam_replay: %s\n", e.what());
        return 1;
    }
}
