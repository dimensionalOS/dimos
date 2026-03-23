// nav_cmd_publisher.cpp — pybind11 module for publishing /NAV_CMD via drdds.
//
// Bypasses rclpy entirely: uses the host's libdrdds.so (FastDDS 2.14)
// to publish NavCmd messages directly. This avoids the glibc 2.32 dependency
// that rclpy (compiled for Ubuntu 22.04 Humble) requires.
//
// Build on NOS (aarch64, Ubuntu 20.04):
//   g++ -O2 -shared -std=c++17 -fPIC \
//       $(python3 -m pybind11 --includes) \
//       -I/usr/local/include \
//       nav_cmd_publisher.cpp \
//       -o nav_cmd_pub$(python3-config --extension-suffix) \
//       -L/usr/local/lib -ldrdds -lfastrtps -lfastcdr
//
// Usage from Python:
//   import nav_cmd_pub
//   pub = nav_cmd_pub.NavCmdPublisher()  # calls DrDDSManager::Init + creates publisher
//   pub.publish(0.5, 0.0, 0.1)          # x_vel, y_vel, yaw_vel
//   pub.shutdown()                       # cleanup

#include <pybind11/pybind11.h>
#include <chrono>
#include <vector>
#include <string>
#include <stdexcept>

#include "drdds/core/drdds_core.h"
#include "dridl/dr_msgs/msg/NavCmd.h"
#include "dridl/dr_msgs/msg/NavCmdPubSubTypes.h"

namespace py = pybind11;

using NavCmdType = drdds::msg::NavCmd;
using NavCmdPubSubType = drdds::msg::NavCmdPubSubType;

class NavCmdPublisher {
public:
    NavCmdPublisher(const std::string& topic = "/NAV_CMD",
                    int domain_id = 0,
                    const std::string& module_id = "dimos_nav_cmd",
                    const std::string& node_name = "nav_cmd_pub") {
        // Initialize drdds (multi-domain Init, same pattern as rsdriver/drdds_recv)
        std::vector<int> domains = {domain_id};
        DrDDSManager::Init(domains, module_id, node_name, false, false, false);

        // Create publisher
        publisher_ = new DrDDSPublisher<NavCmdPubSubType>(topic, domain_id, "");
        if (!publisher_) {
            throw std::runtime_error("Failed to create DrDDSPublisher for " + topic);
        }
    }

    ~NavCmdPublisher() {
        shutdown();
    }

    bool publish(float x_vel, float y_vel, float yaw_vel) {
        if (!publisher_) return false;

        NavCmdType* msg = publisher_->data_;

        // Populate header timestamp
        auto now = std::chrono::system_clock::now();
        auto epoch = now.time_since_epoch();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(epoch);
        auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch) -
                    std::chrono::duration_cast<std::chrono::nanoseconds>(sec);

        msg->header().frame_id(0);
        msg->header().timestamp().sec(static_cast<int32_t>(sec.count()));
        msg->header().timestamp().nsec(static_cast<uint32_t>(nsec.count()));

        // Populate velocity
        msg->data().x_vel(x_vel);
        msg->data().y_vel(y_vel);
        msg->data().yaw_vel(yaw_vel);

        return publisher_->Write(msg);
    }

    int matched_count() const {
        if (!publisher_) return 0;
        return publisher_->GetMatchedCount();
    }

    void shutdown() {
        if (publisher_) {
            delete publisher_;
            publisher_ = nullptr;
        }
    }

private:
    DrDDSPublisher<NavCmdPubSubType>* publisher_ = nullptr;
};

PYBIND11_MODULE(nav_cmd_pub, m) {
    m.doc() = "Native /NAV_CMD publisher using drdds (no rclpy dependency)";

    py::class_<NavCmdPublisher>(m, "NavCmdPublisher")
        .def(py::init<const std::string&, int, const std::string&, const std::string&>(),
             py::arg("topic") = "/NAV_CMD",
             py::arg("domain_id") = 0,
             py::arg("module_id") = "dimos_nav_cmd",
             py::arg("node_name") = "nav_cmd_pub")
        .def("publish", &NavCmdPublisher::publish,
             py::arg("x_vel"), py::arg("y_vel"), py::arg("yaw_vel"),
             "Publish a NavCmd message. Returns True on success.")
        .def("matched_count", &NavCmdPublisher::matched_count,
             "Number of matched DDS subscribers.")
        .def("shutdown", &NavCmdPublisher::shutdown,
             "Destroy the publisher and clean up DDS resources.");
}
