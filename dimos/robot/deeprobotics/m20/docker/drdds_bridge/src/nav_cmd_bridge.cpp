// nav_cmd_bridge.cpp — TCP→drdds bridge for /NAV_CMD on AOS.
//
// Listens on a TCP port for velocity commands from NOS, publishes them
// to /NAV_CMD via drdds locally on AOS. This bridges the gap between
// NOS (where dimos runs) and AOS (where ctrlmcu subscribes to /NAV_CMD),
// since drdds uses SHM-only transport which can't cross machine boundaries.
//
// Protocol: 12-byte binary messages (3 x float32: x_vel, y_vel, yaw_vel)
// sent over TCP from NOS to AOS port 9740.
//
// Build on AOS:
//   g++ -O2 -std=c++17 nav_cmd_bridge.cpp \
//       -I/usr/local/include -I/usr/local/include/dridl \
//       $(find /usr/local/include/dridl -mindepth 1 -maxdepth 1 -type d -exec echo -n "-I{}/msg " \;) \
//       -o nav_cmd_bridge -L/usr/local/lib -ldrdds -lfastrtps -lfastcdr -lpthread
//
// Run: ./nav_cmd_bridge [port] (default: 9740)

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "drdds/core/drdds_core.h"
#include "dridl/dr_msgs/msg/NavCmd.h"
#include "dridl/dr_msgs/msg/NavCmdPubSubTypes.h"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static constexpr int DEFAULT_PORT = 9740;
static constexpr int MSG_SIZE = 12;  // 3 x float32

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    int port = (argc > 1) ? std::atoi(argv[1]) : DEFAULT_PORT;

    // Initialize drdds — local SHM on AOS, where ctrlmcu also runs.
    // Use DrDDSChannel (pub+sub) instead of DrDDSPublisher alone — this is
    // required for proper DDS endpoint matching (same pattern as drdds_recv).
    std::vector<int> domains = {0};
    DrDDSManager::Init(domains, "nav_cmd_bridge", "nav_cmd_bridge", false, false, false);

    DrDDSChannel<drdds::msg::NavCmdPubSubType> nav_cmd_ch("/NAV_CMD", 0);
    auto* publisher = nav_cmd_ch.GetPublisher();
    if (!publisher) {
        fprintf(stderr, "[nav_cmd_bridge] Failed to create /NAV_CMD channel\n");
        return 1;
    }

    // Wait for subscriber match
    fprintf(stderr, "[nav_cmd_bridge] Waiting for /NAV_CMD subscriber...\n");
    for (int i = 0; i < 30 && g_running; i++) {
        if (publisher->GetMatchedCount() > 0) break;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    fprintf(stderr, "[nav_cmd_bridge] matched_count=%d\n", publisher->GetMatchedCount());

    // Create TCP server
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("[nav_cmd_bridge] socket");
        return 1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[nav_cmd_bridge] bind");
        close(server_fd);
        return 1;
    }

    listen(server_fd, 1);
    fprintf(stderr, "[nav_cmd_bridge] Listening on port %d for velocity commands\n", port);

    while (g_running) {
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);

        // Accept with timeout so we can check g_running
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(server_fd, &fds);
        struct timeval tv{1, 0};  // 1s timeout
        if (select(server_fd + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;

        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) continue;

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
        fprintf(stderr, "[nav_cmd_bridge] Client connected from %s\n", client_ip);

        uint64_t msg_count = 0;
        while (g_running) {
            uint8_t buf[MSG_SIZE];
            int n = recv(client_fd, buf, MSG_SIZE, MSG_WAITALL);
            if (n != MSG_SIZE) break;

            float x_vel, y_vel, yaw_vel;
            std::memcpy(&x_vel, buf, 4);
            std::memcpy(&y_vel, buf + 4, 4);
            std::memcpy(&yaw_vel, buf + 8, 4);

            // Publish to /NAV_CMD
            auto* msg = publisher->data_;
            auto now = std::chrono::system_clock::now();
            auto epoch = now.time_since_epoch();
            auto sec = std::chrono::duration_cast<std::chrono::seconds>(epoch);
            auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch) -
                        std::chrono::duration_cast<std::chrono::nanoseconds>(sec);

            msg->header().frame_id(0);
            msg->header().timestamp().sec(static_cast<int32_t>(sec.count()));
            msg->header().timestamp().nsec(static_cast<uint32_t>(nsec.count()));
            msg->data().x_vel(x_vel);
            msg->data().y_vel(y_vel);
            msg->data().yaw_vel(yaw_vel);

            publisher->Write(msg);
            msg_count++;

            if (msg_count % 100 == 1) {
                fprintf(stderr, "[nav_cmd_bridge] #%lu x=%.3f y=%.3f yaw=%.3f matched=%d\n",
                        msg_count, x_vel, y_vel, yaw_vel, publisher->GetMatchedCount());
            }
        }

        close(client_fd);
        fprintf(stderr, "[nav_cmd_bridge] Client disconnected (%lu msgs)\n", msg_count);
    }

    close(server_fd);
    delete publisher;
    fprintf(stderr, "[nav_cmd_bridge] Shutdown\n");
    return 0;
}
