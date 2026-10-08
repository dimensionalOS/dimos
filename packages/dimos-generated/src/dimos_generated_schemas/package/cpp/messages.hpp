// Generated from ROS2 .msg definitions. Do not edit.
#pragma once
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <fastcdr/Cdr.h>
#include <fastcdr/CdrSizeCalculator.hpp>
#include "dimos_cdr.hpp"
#ifndef DIMOS_MESSAGE_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C_TYPE
#define DIMOS_MESSAGE_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C_TYPE
namespace builtin_interfaces::msg {
struct Duration {
int32_t sec{};
uint32_t nanosec{};
bool operator==(const Duration& other) const { return this->sec == other.sec && this->nanosec == other.nanosec; }
bool operator!=(const Duration& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "builtin_interfaces/msg/Duration";
};
}
#endif
#ifndef DIMOS_MESSAGE_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4_TYPE
#define DIMOS_MESSAGE_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4_TYPE
namespace builtin_interfaces::msg {
struct Time {
int32_t sec{};
uint32_t nanosec{};
bool operator==(const Time& other) const { return this->sec == other.sec && this->nanosec == other.nanosec; }
bool operator!=(const Time& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "builtin_interfaces/msg/Time";
};
}
#endif
#ifndef DIMOS_MESSAGE_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D_TYPE
#define DIMOS_MESSAGE_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D_TYPE
namespace std_msgs::msg {
struct Header {
builtin_interfaces::msg::Time stamp{};
std::string frame_id{};
bool operator==(const Header& other) const { return this->stamp == other.stamp && this->frame_id == other.frame_id; }
bool operator!=(const Header& other) const { return !(*this == other); }
void validate() const {
stamp.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Header";
};
}
#endif
#ifndef DIMOS_MESSAGE_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51_TYPE
#define DIMOS_MESSAGE_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51_TYPE
namespace vision_msgs::msg {
struct Point2D {
double x{};
double y{};
bool operator==(const Point2D& other) const { return this->x == other.x && this->y == other.y; }
bool operator!=(const Point2D& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "vision_msgs/msg/Point2D";
};
}
#endif
#ifndef DIMOS_MESSAGE_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97_TYPE
#define DIMOS_MESSAGE_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97_TYPE
namespace vision_msgs::msg {
struct Pose2D {
vision_msgs::msg::Point2D position{};
double theta{};
bool operator==(const Pose2D& other) const { return this->position == other.position && this->theta == other.theta; }
bool operator!=(const Pose2D& other) const { return !(*this == other); }
void validate() const {
position.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/Pose2D";
};
}
#endif
#ifndef DIMOS_MESSAGE_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA_TYPE
#define DIMOS_MESSAGE_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA_TYPE
namespace vision_msgs::msg {
struct BoundingBox2D {
vision_msgs::msg::Pose2D center{};
double size_x{};
double size_y{};
bool operator==(const BoundingBox2D& other) const { return this->center == other.center && this->size_x == other.size_x && this->size_y == other.size_y; }
bool operator!=(const BoundingBox2D& other) const { return !(*this == other); }
void validate() const {
center.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/BoundingBox2D";
};
}
#endif
#ifndef DIMOS_MESSAGE_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A_TYPE
#define DIMOS_MESSAGE_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A_TYPE
namespace dimos_msgs::msg {
struct BoundingBox2DArray {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::BoundingBox2D> boxes{};
bool operator==(const BoundingBox2DArray& other) const { return this->header == other.header && this->boxes == other.boxes; }
bool operator!=(const BoundingBox2DArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : boxes) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/BoundingBox2DArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C_TYPE
#define DIMOS_MESSAGE_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C_TYPE
namespace geometry_msgs::msg {
struct Point {
double x{};
double y{};
double z{};
bool operator==(const Point& other) const { return this->x == other.x && this->y == other.y && this->z == other.z; }
bool operator!=(const Point& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "geometry_msgs/msg/Point";
};
}
#endif
#ifndef DIMOS_MESSAGE_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD_TYPE
#define DIMOS_MESSAGE_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD_TYPE
namespace geometry_msgs::msg {
struct Quaternion {
double x{0.0};
double y{0.0};
double z{0.0};
double w{1.0};
bool operator==(const Quaternion& other) const { return this->x == other.x && this->y == other.y && this->z == other.z && this->w == other.w; }
bool operator!=(const Quaternion& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "geometry_msgs/msg/Quaternion";
};
}
#endif
#ifndef DIMOS_MESSAGE_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827_TYPE
#define DIMOS_MESSAGE_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827_TYPE
namespace geometry_msgs::msg {
struct Pose {
geometry_msgs::msg::Point position{};
geometry_msgs::msg::Quaternion orientation{};
bool operator==(const Pose& other) const { return this->position == other.position && this->orientation == other.orientation; }
bool operator!=(const Pose& other) const { return !(*this == other); }
void validate() const {
position.validate();
orientation.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Pose";
};
}
#endif
#ifndef DIMOS_MESSAGE_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02_TYPE
#define DIMOS_MESSAGE_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02_TYPE
namespace geometry_msgs::msg {
struct Vector3 {
double x{};
double y{};
double z{};
bool operator==(const Vector3& other) const { return this->x == other.x && this->y == other.y && this->z == other.z; }
bool operator!=(const Vector3& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "geometry_msgs/msg/Vector3";
};
}
#endif
#ifndef DIMOS_MESSAGE_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0_TYPE
#define DIMOS_MESSAGE_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0_TYPE
namespace vision_msgs::msg {
struct BoundingBox3D {
geometry_msgs::msg::Pose center{};
geometry_msgs::msg::Vector3 size{};
bool operator==(const BoundingBox3D& other) const { return this->center == other.center && this->size == other.size; }
bool operator!=(const BoundingBox3D& other) const { return !(*this == other); }
void validate() const {
center.validate();
size.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/BoundingBox3D";
};
}
#endif
#ifndef DIMOS_MESSAGE_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A_TYPE
#define DIMOS_MESSAGE_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A_TYPE
namespace dimos_msgs::msg {
struct BoundingBox3DArray {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::BoundingBox3D> boxes{};
bool operator==(const BoundingBox3DArray& other) const { return this->header == other.header && this->boxes == other.boxes; }
bool operator!=(const BoundingBox3DArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : boxes) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/BoundingBox3DArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87_TYPE
#define DIMOS_MESSAGE_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87_TYPE
namespace dimos_msgs::msg {
struct EntityMarker {
std::string entity_id{};
std::string label{};
std::string entity_type{};
geometry_msgs::msg::Point position{};
bool operator==(const EntityMarker& other) const { return this->entity_id == other.entity_id && this->label == other.label && this->entity_type == other.entity_type && this->position == other.position; }
bool operator!=(const EntityMarker& other) const { return !(*this == other); }
void validate() const {
position.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/EntityMarker";
};
}
#endif
#ifndef DIMOS_MESSAGE_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254_TYPE
#define DIMOS_MESSAGE_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254_TYPE
namespace dimos_msgs::msg {
struct EntityMarkers {
std_msgs::msg::Header header{};
std::vector<dimos_msgs::msg::EntityMarker> markers{};
bool operator==(const EntityMarkers& other) const { return this->header == other.header && this->markers == other.markers; }
bool operator!=(const EntityMarkers& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : markers) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/EntityMarkers";
};
}
#endif
#ifndef DIMOS_MESSAGE_78F4ED1DEB1B34DFDA6F24F5EE35F865A25466476BAECF67ABEE6DEBC2A7E65B_TYPE
#define DIMOS_MESSAGE_78F4ED1DEB1B34DFDA6F24F5EE35F865A25466476BAECF67ABEE6DEBC2A7E65B_TYPE
namespace dimos_msgs::msg {
struct EpisodeStatus {
double ts{};
std::string state{};
int64_t episodes_saved{};
int64_t episodes_discarded{};
std::string last_event{"init"};
std::vector<std::string> task_label{};
bool operator==(const EpisodeStatus& other) const { return this->ts == other.ts && this->state == other.state && this->episodes_saved == other.episodes_saved && this->episodes_discarded == other.episodes_discarded && this->last_event == other.last_event && this->task_label == other.task_label; }
bool operator!=(const EpisodeStatus& other) const { return !(*this == other); }
void validate() const {
if (task_label.size() > 1) throw std::length_error("task_label exceeds sequence bound");
}
static constexpr const char* msg_name = "dimos_msgs/msg/EpisodeStatus";
};
}
#endif
#ifndef DIMOS_MESSAGE_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6_TYPE
#define DIMOS_MESSAGE_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6_TYPE
namespace dimos_msgs::msg {
struct GraspCandidate {
geometry_msgs::msg::Pose pose{};
double score{};
bool operator==(const GraspCandidate& other) const { return this->pose == other.pose && this->score == other.score; }
bool operator!=(const GraspCandidate& other) const { return !(*this == other); }
void validate() const {
pose.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/GraspCandidate";
};
}
#endif
#ifndef DIMOS_MESSAGE_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC_TYPE
#define DIMOS_MESSAGE_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC_TYPE
namespace dimos_msgs::msg {
struct GraspCandidateArray {
std_msgs::msg::Header header{};
std::vector<dimos_msgs::msg::GraspCandidate> candidates{};
bool operator==(const GraspCandidateArray& other) const { return this->header == other.header && this->candidates == other.candidates; }
bool operator!=(const GraspCandidateArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : candidates) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/GraspCandidateArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533_TYPE
#define DIMOS_MESSAGE_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533_TYPE
namespace dimos_msgs::msg {
struct ImuInfo {
std_msgs::msg::Header header{};
double gyro_noise_density{};
double gyro_random_walk{};
double accel_noise_density{};
double accel_random_walk{};
double frequency{};
bool operator==(const ImuInfo& other) const { return this->header == other.header && this->gyro_noise_density == other.gyro_noise_density && this->gyro_random_walk == other.gyro_random_walk && this->accel_noise_density == other.accel_noise_density && this->accel_random_walk == other.accel_random_walk && this->frequency == other.frequency; }
bool operator!=(const ImuInfo& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/ImuInfo";
};
}
#endif
#ifndef DIMOS_MESSAGE_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02_TYPE
#define DIMOS_MESSAGE_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02_TYPE
namespace dimos_msgs::msg {
struct JointCommand {
std_msgs::msg::Header header{};
std::vector<double> positions{};
bool operator==(const JointCommand& other) const { return this->header == other.header && this->positions == other.positions; }
bool operator!=(const JointCommand& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/JointCommand";
};
}
#endif
#ifndef DIMOS_MESSAGE_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2_TYPE
#define DIMOS_MESSAGE_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2_TYPE
namespace dimos_msgs::msg {
struct LineSegment3D {
geometry_msgs::msg::Point start{};
geometry_msgs::msg::Point end{};
double weight{1.0};
bool operator==(const LineSegment3D& other) const { return this->start == other.start && this->end == other.end && this->weight == other.weight; }
bool operator!=(const LineSegment3D& other) const { return !(*this == other); }
void validate() const {
start.validate();
end.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/LineSegment3D";
};
}
#endif
#ifndef DIMOS_MESSAGE_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C_TYPE
#define DIMOS_MESSAGE_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C_TYPE
namespace dimos_msgs::msg {
struct LineSegments3D {
std_msgs::msg::Header header{};
std::vector<dimos_msgs::msg::LineSegment3D> segments{};
bool operator==(const LineSegments3D& other) const { return this->header == other.header && this->segments == other.segments; }
bool operator!=(const LineSegments3D& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : segments) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/LineSegments3D";
};
}
#endif
#ifndef DIMOS_MESSAGE_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03_TYPE
#define DIMOS_MESSAGE_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03_TYPE
namespace dimos_msgs::msg {
struct MotorCommandArray {
std_msgs::msg::Header header{};
std::vector<double> q{};
std::vector<double> dq{};
std::vector<double> kp{};
std::vector<double> kd{};
std::vector<double> tau{};
bool operator==(const MotorCommandArray& other) const { return this->header == other.header && this->q == other.q && this->dq == other.dq && this->kp == other.kp && this->kd == other.kd && this->tau == other.tau; }
bool operator!=(const MotorCommandArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/MotorCommandArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3_TYPE
#define DIMOS_MESSAGE_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3_TYPE
namespace dimos_msgs::msg {
struct RobotState {
std_msgs::msg::Header header{};
int32_t state{};
int32_t mode{};
int32_t error_code{};
int32_t warn_code{};
int32_t cmdnum{};
int32_t mt_brake{};
int32_t mt_able{};
std::vector<double> tcp_pose{};
std::vector<double> tcp_offset{};
std::vector<double> joints{};
bool operator==(const RobotState& other) const { return this->header == other.header && this->state == other.state && this->mode == other.mode && this->error_code == other.error_code && this->warn_code == other.warn_code && this->cmdnum == other.cmdnum && this->mt_brake == other.mt_brake && this->mt_able == other.mt_able && this->tcp_pose == other.tcp_pose && this->tcp_offset == other.tcp_offset && this->joints == other.joints; }
bool operator!=(const RobotState& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/RobotState";
};
}
#endif
#ifndef DIMOS_MESSAGE_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64_TYPE
#define DIMOS_MESSAGE_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64_TYPE
namespace dimos_msgs::msg {
struct TrajectoryStatus {
static constexpr uint8_t IDLE = 0;
static constexpr uint8_t EXECUTING = 1;
static constexpr uint8_t COMPLETED = 2;
static constexpr uint8_t ABORTED = 3;
static constexpr uint8_t FAULT = 4;
std_msgs::msg::Header header{};
uint8_t state{};
double progress{};
builtin_interfaces::msg::Duration time_elapsed{};
builtin_interfaces::msg::Duration time_remaining{};
std::string error{};
bool operator==(const TrajectoryStatus& other) const { return this->header == other.header && this->state == other.state && this->progress == other.progress && this->time_elapsed == other.time_elapsed && this->time_remaining == other.time_remaining && this->error == other.error; }
bool operator!=(const TrajectoryStatus& other) const { return !(*this == other); }
void validate() const {
header.validate();
time_elapsed.validate();
time_remaining.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/TrajectoryStatus";
};
}
#endif
#ifndef DIMOS_MESSAGE_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125_TYPE
#define DIMOS_MESSAGE_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125_TYPE
namespace dimos_msgs::msg {
struct VideoStats {
std_msgs::msg::Header header{};
double fps{};
double kbps{};
uint32_t width{};
uint32_t height{};
double loss_pct{};
double jitter_buffer_ms{};
double decode_ms{};
uint64_t frames_dropped{};
uint64_t freezes{};
double e2e_latency_ms{};
bool operator==(const VideoStats& other) const { return this->header == other.header && this->fps == other.fps && this->kbps == other.kbps && this->width == other.width && this->height == other.height && this->loss_pct == other.loss_pct && this->jitter_buffer_ms == other.jitter_buffer_ms && this->decode_ms == other.decode_ms && this->frames_dropped == other.frames_dropped && this->freezes == other.freezes && this->e2e_latency_ms == other.e2e_latency_ms; }
bool operator!=(const VideoStats& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/VideoStats";
};
}
#endif
#ifndef DIMOS_MESSAGE_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82_TYPE
#define DIMOS_MESSAGE_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82_TYPE
namespace foxglove_msgs::msg {
struct CompressedVideo {
builtin_interfaces::msg::Time timestamp{};
std::string frame_id{};
std::vector<uint8_t> data{};
std::string format{};
bool operator==(const CompressedVideo& other) const { return this->timestamp == other.timestamp && this->frame_id == other.frame_id && this->data == other.data && this->format == other.format; }
bool operator!=(const CompressedVideo& other) const { return !(*this == other); }
void validate() const {
timestamp.validate();
}
static constexpr const char* msg_name = "foxglove_msgs/msg/CompressedVideo";
};
}
#endif
#ifndef DIMOS_MESSAGE_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962_TYPE
#define DIMOS_MESSAGE_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962_TYPE
namespace geometry_msgs::msg {
struct Accel {
geometry_msgs::msg::Vector3 linear{};
geometry_msgs::msg::Vector3 angular{};
bool operator==(const Accel& other) const { return this->linear == other.linear && this->angular == other.angular; }
bool operator!=(const Accel& other) const { return !(*this == other); }
void validate() const {
linear.validate();
angular.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Accel";
};
}
#endif
#ifndef DIMOS_MESSAGE_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3_TYPE
#define DIMOS_MESSAGE_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3_TYPE
namespace geometry_msgs::msg {
struct AccelStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Accel accel{};
bool operator==(const AccelStamped& other) const { return this->header == other.header && this->accel == other.accel; }
bool operator!=(const AccelStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
accel.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/AccelStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7_TYPE
#define DIMOS_MESSAGE_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7_TYPE
namespace geometry_msgs::msg {
struct AccelWithCovariance {
geometry_msgs::msg::Accel accel{};
std::array<double, 36> covariance{};
bool operator==(const AccelWithCovariance& other) const { return this->accel == other.accel && this->covariance == other.covariance; }
bool operator!=(const AccelWithCovariance& other) const { return !(*this == other); }
void validate() const {
accel.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/AccelWithCovariance";
};
}
#endif
#ifndef DIMOS_MESSAGE_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283_TYPE
#define DIMOS_MESSAGE_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283_TYPE
namespace geometry_msgs::msg {
struct AccelWithCovarianceStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::AccelWithCovariance accel{};
bool operator==(const AccelWithCovarianceStamped& other) const { return this->header == other.header && this->accel == other.accel; }
bool operator!=(const AccelWithCovarianceStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
accel.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/AccelWithCovarianceStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE_TYPE
#define DIMOS_MESSAGE_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE_TYPE
namespace geometry_msgs::msg {
struct Inertia {
double m{};
geometry_msgs::msg::Vector3 com{};
double ixx{};
double ixy{};
double ixz{};
double iyy{};
double iyz{};
double izz{};
bool operator==(const Inertia& other) const { return this->m == other.m && this->com == other.com && this->ixx == other.ixx && this->ixy == other.ixy && this->ixz == other.ixz && this->iyy == other.iyy && this->iyz == other.iyz && this->izz == other.izz; }
bool operator!=(const Inertia& other) const { return !(*this == other); }
void validate() const {
com.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Inertia";
};
}
#endif
#ifndef DIMOS_MESSAGE_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF_TYPE
#define DIMOS_MESSAGE_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF_TYPE
namespace geometry_msgs::msg {
struct InertiaStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Inertia inertia{};
bool operator==(const InertiaStamped& other) const { return this->header == other.header && this->inertia == other.inertia; }
bool operator!=(const InertiaStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
inertia.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/InertiaStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6_TYPE
#define DIMOS_MESSAGE_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6_TYPE
namespace geometry_msgs::msg {
struct Point32 {
float x{};
float y{};
float z{};
bool operator==(const Point32& other) const { return this->x == other.x && this->y == other.y && this->z == other.z; }
bool operator!=(const Point32& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "geometry_msgs/msg/Point32";
};
}
#endif
#ifndef DIMOS_MESSAGE_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328_TYPE
#define DIMOS_MESSAGE_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328_TYPE
namespace geometry_msgs::msg {
struct PointStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Point point{};
bool operator==(const PointStamped& other) const { return this->header == other.header && this->point == other.point; }
bool operator!=(const PointStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
point.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PointStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469_TYPE
#define DIMOS_MESSAGE_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469_TYPE
namespace geometry_msgs::msg {
struct Polygon {
std::vector<geometry_msgs::msg::Point32> points{};
bool operator==(const Polygon& other) const { return this->points == other.points; }
bool operator!=(const Polygon& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : points) { item.validate(); }
}
static constexpr const char* msg_name = "geometry_msgs/msg/Polygon";
};
}
#endif
#ifndef DIMOS_MESSAGE_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C_TYPE
#define DIMOS_MESSAGE_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C_TYPE
namespace geometry_msgs::msg {
struct PolygonInstance {
geometry_msgs::msg::Polygon polygon{};
int64_t id{};
bool operator==(const PolygonInstance& other) const { return this->polygon == other.polygon && this->id == other.id; }
bool operator!=(const PolygonInstance& other) const { return !(*this == other); }
void validate() const {
polygon.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PolygonInstance";
};
}
#endif
#ifndef DIMOS_MESSAGE_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22_TYPE
#define DIMOS_MESSAGE_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22_TYPE
namespace geometry_msgs::msg {
struct PolygonInstanceStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::PolygonInstance polygon{};
bool operator==(const PolygonInstanceStamped& other) const { return this->header == other.header && this->polygon == other.polygon; }
bool operator!=(const PolygonInstanceStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
polygon.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PolygonInstanceStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F_TYPE
#define DIMOS_MESSAGE_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F_TYPE
namespace geometry_msgs::msg {
struct PolygonStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Polygon polygon{};
bool operator==(const PolygonStamped& other) const { return this->header == other.header && this->polygon == other.polygon; }
bool operator!=(const PolygonStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
polygon.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PolygonStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8_TYPE
#define DIMOS_MESSAGE_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8_TYPE
namespace geometry_msgs::msg {
struct Pose2D {
double x{};
double y{};
double theta{};
bool operator==(const Pose2D& other) const { return this->x == other.x && this->y == other.y && this->theta == other.theta; }
bool operator!=(const Pose2D& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "geometry_msgs/msg/Pose2D";
};
}
#endif
#ifndef DIMOS_MESSAGE_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3_TYPE
#define DIMOS_MESSAGE_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3_TYPE
namespace geometry_msgs::msg {
struct PoseArray {
std_msgs::msg::Header header{};
std::vector<geometry_msgs::msg::Pose> poses{};
bool operator==(const PoseArray& other) const { return this->header == other.header && this->poses == other.poses; }
bool operator!=(const PoseArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : poses) { item.validate(); }
}
static constexpr const char* msg_name = "geometry_msgs/msg/PoseArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F_TYPE
#define DIMOS_MESSAGE_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F_TYPE
namespace geometry_msgs::msg {
struct PoseStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Pose pose{};
bool operator==(const PoseStamped& other) const { return this->header == other.header && this->pose == other.pose; }
bool operator!=(const PoseStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
pose.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PoseStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A_TYPE
#define DIMOS_MESSAGE_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A_TYPE
namespace geometry_msgs::msg {
struct PoseWithCovariance {
geometry_msgs::msg::Pose pose{};
std::array<double, 36> covariance{};
bool operator==(const PoseWithCovariance& other) const { return this->pose == other.pose && this->covariance == other.covariance; }
bool operator!=(const PoseWithCovariance& other) const { return !(*this == other); }
void validate() const {
pose.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PoseWithCovariance";
};
}
#endif
#ifndef DIMOS_MESSAGE_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B_TYPE
#define DIMOS_MESSAGE_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B_TYPE
namespace geometry_msgs::msg {
struct PoseWithCovarianceStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::PoseWithCovariance pose{};
bool operator==(const PoseWithCovarianceStamped& other) const { return this->header == other.header && this->pose == other.pose; }
bool operator!=(const PoseWithCovarianceStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
pose.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PoseWithCovarianceStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5_TYPE
#define DIMOS_MESSAGE_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5_TYPE
namespace geometry_msgs::msg {
struct QuaternionStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Quaternion quaternion{};
bool operator==(const QuaternionStamped& other) const { return this->header == other.header && this->quaternion == other.quaternion; }
bool operator!=(const QuaternionStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
quaternion.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/QuaternionStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238_TYPE
#define DIMOS_MESSAGE_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238_TYPE
namespace geometry_msgs::msg {
struct Transform {
geometry_msgs::msg::Vector3 translation{};
geometry_msgs::msg::Quaternion rotation{};
bool operator==(const Transform& other) const { return this->translation == other.translation && this->rotation == other.rotation; }
bool operator!=(const Transform& other) const { return !(*this == other); }
void validate() const {
translation.validate();
rotation.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Transform";
};
}
#endif
#ifndef DIMOS_MESSAGE_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA_TYPE
#define DIMOS_MESSAGE_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA_TYPE
namespace geometry_msgs::msg {
struct TransformStamped {
std_msgs::msg::Header header{};
std::string child_frame_id{};
geometry_msgs::msg::Transform transform{};
bool operator==(const TransformStamped& other) const { return this->header == other.header && this->child_frame_id == other.child_frame_id && this->transform == other.transform; }
bool operator!=(const TransformStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
transform.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/TransformStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85_TYPE
#define DIMOS_MESSAGE_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85_TYPE
namespace geometry_msgs::msg {
struct Twist {
geometry_msgs::msg::Vector3 linear{};
geometry_msgs::msg::Vector3 angular{};
bool operator==(const Twist& other) const { return this->linear == other.linear && this->angular == other.angular; }
bool operator!=(const Twist& other) const { return !(*this == other); }
void validate() const {
linear.validate();
angular.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Twist";
};
}
#endif
#ifndef DIMOS_MESSAGE_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C_TYPE
#define DIMOS_MESSAGE_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C_TYPE
namespace geometry_msgs::msg {
struct TwistStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Twist twist{};
bool operator==(const TwistStamped& other) const { return this->header == other.header && this->twist == other.twist; }
bool operator!=(const TwistStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
twist.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/TwistStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD_TYPE
#define DIMOS_MESSAGE_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD_TYPE
namespace geometry_msgs::msg {
struct TwistWithCovariance {
geometry_msgs::msg::Twist twist{};
std::array<double, 36> covariance{};
bool operator==(const TwistWithCovariance& other) const { return this->twist == other.twist && this->covariance == other.covariance; }
bool operator!=(const TwistWithCovariance& other) const { return !(*this == other); }
void validate() const {
twist.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/TwistWithCovariance";
};
}
#endif
#ifndef DIMOS_MESSAGE_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6_TYPE
#define DIMOS_MESSAGE_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6_TYPE
namespace geometry_msgs::msg {
struct TwistWithCovarianceStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::TwistWithCovariance twist{};
bool operator==(const TwistWithCovarianceStamped& other) const { return this->header == other.header && this->twist == other.twist; }
bool operator!=(const TwistWithCovarianceStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
twist.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/TwistWithCovarianceStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75_TYPE
#define DIMOS_MESSAGE_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75_TYPE
namespace geometry_msgs::msg {
struct Vector3Stamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Vector3 vector{};
bool operator==(const Vector3Stamped& other) const { return this->header == other.header && this->vector == other.vector; }
bool operator!=(const Vector3Stamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
vector.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Vector3Stamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387_TYPE
#define DIMOS_MESSAGE_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387_TYPE
namespace geometry_msgs::msg {
struct VelocityStamped {
std_msgs::msg::Header header{};
std::string body_frame_id{};
std::string reference_frame_id{};
geometry_msgs::msg::Twist velocity{};
bool operator==(const VelocityStamped& other) const { return this->header == other.header && this->body_frame_id == other.body_frame_id && this->reference_frame_id == other.reference_frame_id && this->velocity == other.velocity; }
bool operator!=(const VelocityStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
velocity.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/VelocityStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732_TYPE
#define DIMOS_MESSAGE_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732_TYPE
namespace geometry_msgs::msg {
struct VelocityWithCovarianceStamped {
std_msgs::msg::Header header{};
std::string body_frame_id{};
std::string reference_frame_id{};
geometry_msgs::msg::TwistWithCovariance velocity{};
bool operator==(const VelocityWithCovarianceStamped& other) const { return this->header == other.header && this->body_frame_id == other.body_frame_id && this->reference_frame_id == other.reference_frame_id && this->velocity == other.velocity; }
bool operator!=(const VelocityWithCovarianceStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
velocity.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/VelocityWithCovarianceStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12_TYPE
#define DIMOS_MESSAGE_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12_TYPE
namespace geometry_msgs::msg {
struct Wrench {
geometry_msgs::msg::Vector3 force{};
geometry_msgs::msg::Vector3 torque{};
bool operator==(const Wrench& other) const { return this->force == other.force && this->torque == other.torque; }
bool operator!=(const Wrench& other) const { return !(*this == other); }
void validate() const {
force.validate();
torque.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Wrench";
};
}
#endif
#ifndef DIMOS_MESSAGE_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB_TYPE
#define DIMOS_MESSAGE_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB_TYPE
namespace geometry_msgs::msg {
struct WrenchStamped {
std_msgs::msg::Header header{};
geometry_msgs::msg::Wrench wrench{};
bool operator==(const WrenchStamped& other) const { return this->header == other.header && this->wrench == other.wrench; }
bool operator!=(const WrenchStamped& other) const { return !(*this == other); }
void validate() const {
header.validate();
wrench.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/WrenchStamped";
};
}
#endif
#ifndef DIMOS_MESSAGE_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0_TYPE
#define DIMOS_MESSAGE_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0_TYPE
namespace nav_msgs::msg {
struct Goals {
std_msgs::msg::Header header{};
std::vector<geometry_msgs::msg::PoseStamped> goals{};
bool operator==(const Goals& other) const { return this->header == other.header && this->goals == other.goals; }
bool operator!=(const Goals& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : goals) { item.validate(); }
}
static constexpr const char* msg_name = "nav_msgs/msg/Goals";
};
}
#endif
#ifndef DIMOS_MESSAGE_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B_TYPE
#define DIMOS_MESSAGE_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B_TYPE
namespace nav_msgs::msg {
struct GridCells {
std_msgs::msg::Header header{};
float cell_width{};
float cell_height{};
std::vector<geometry_msgs::msg::Point> cells{};
bool operator==(const GridCells& other) const { return this->header == other.header && this->cell_width == other.cell_width && this->cell_height == other.cell_height && this->cells == other.cells; }
bool operator!=(const GridCells& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : cells) { item.validate(); }
}
static constexpr const char* msg_name = "nav_msgs/msg/GridCells";
};
}
#endif
#ifndef DIMOS_MESSAGE_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0_TYPE
#define DIMOS_MESSAGE_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0_TYPE
namespace nav_msgs::msg {
struct MapMetaData {
builtin_interfaces::msg::Time map_load_time{};
float resolution{};
uint32_t width{};
uint32_t height{};
geometry_msgs::msg::Pose origin{};
bool operator==(const MapMetaData& other) const { return this->map_load_time == other.map_load_time && this->resolution == other.resolution && this->width == other.width && this->height == other.height && this->origin == other.origin; }
bool operator!=(const MapMetaData& other) const { return !(*this == other); }
void validate() const {
map_load_time.validate();
origin.validate();
}
static constexpr const char* msg_name = "nav_msgs/msg/MapMetaData";
};
}
#endif
#ifndef DIMOS_MESSAGE_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D_TYPE
#define DIMOS_MESSAGE_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D_TYPE
namespace nav_msgs::msg {
struct OccupancyGrid {
std_msgs::msg::Header header{};
nav_msgs::msg::MapMetaData info{};
std::vector<int8_t> data{};
bool operator==(const OccupancyGrid& other) const { return this->header == other.header && this->info == other.info && this->data == other.data; }
bool operator!=(const OccupancyGrid& other) const { return !(*this == other); }
void validate() const {
header.validate();
info.validate();
}
static constexpr const char* msg_name = "nav_msgs/msg/OccupancyGrid";
};
}
#endif
#ifndef DIMOS_MESSAGE_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24_TYPE
#define DIMOS_MESSAGE_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24_TYPE
namespace nav_msgs::msg {
struct Odometry {
std_msgs::msg::Header header{};
std::string child_frame_id{};
geometry_msgs::msg::PoseWithCovariance pose{};
geometry_msgs::msg::TwistWithCovariance twist{};
bool operator==(const Odometry& other) const { return this->header == other.header && this->child_frame_id == other.child_frame_id && this->pose == other.pose && this->twist == other.twist; }
bool operator!=(const Odometry& other) const { return !(*this == other); }
void validate() const {
header.validate();
pose.validate();
twist.validate();
}
static constexpr const char* msg_name = "nav_msgs/msg/Odometry";
};
}
#endif
#ifndef DIMOS_MESSAGE_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7_TYPE
#define DIMOS_MESSAGE_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7_TYPE
namespace nav_msgs::msg {
struct Path {
std_msgs::msg::Header header{};
std::vector<geometry_msgs::msg::PoseStamped> poses{};
bool operator==(const Path& other) const { return this->header == other.header && this->poses == other.poses; }
bool operator!=(const Path& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : poses) { item.validate(); }
}
static constexpr const char* msg_name = "nav_msgs/msg/Path";
};
}
#endif
#ifndef DIMOS_MESSAGE_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96_TYPE
#define DIMOS_MESSAGE_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96_TYPE
namespace nav_msgs::msg {
struct TrajectoryPoint {
std_msgs::msg::Header header{};
geometry_msgs::msg::Pose pose{};
geometry_msgs::msg::Twist velocity{};
geometry_msgs::msg::Accel acceleration{};
geometry_msgs::msg::Wrench effort{};
bool operator==(const TrajectoryPoint& other) const { return this->header == other.header && this->pose == other.pose && this->velocity == other.velocity && this->acceleration == other.acceleration && this->effort == other.effort; }
bool operator!=(const TrajectoryPoint& other) const { return !(*this == other); }
void validate() const {
header.validate();
pose.validate();
velocity.validate();
acceleration.validate();
effort.validate();
}
static constexpr const char* msg_name = "nav_msgs/msg/TrajectoryPoint";
};
}
#endif
#ifndef DIMOS_MESSAGE_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA_TYPE
#define DIMOS_MESSAGE_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA_TYPE
namespace nav_msgs::msg {
struct Trajectory {
std_msgs::msg::Header header{};
std::vector<nav_msgs::msg::TrajectoryPoint> points{};
bool operator==(const Trajectory& other) const { return this->header == other.header && this->points == other.points; }
bool operator!=(const Trajectory& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : points) { item.validate(); }
}
static constexpr const char* msg_name = "nav_msgs/msg/Trajectory";
};
}
#endif
#ifndef DIMOS_MESSAGE_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5_TYPE
#define DIMOS_MESSAGE_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5_TYPE
namespace sensor_msgs::msg {
struct BatteryState {
static constexpr uint8_t POWER_SUPPLY_STATUS_UNKNOWN = 0;
static constexpr uint8_t POWER_SUPPLY_STATUS_CHARGING = 1;
static constexpr uint8_t POWER_SUPPLY_STATUS_DISCHARGING = 2;
static constexpr uint8_t POWER_SUPPLY_STATUS_NOT_CHARGING = 3;
static constexpr uint8_t POWER_SUPPLY_STATUS_FULL = 4;
static constexpr uint8_t POWER_SUPPLY_HEALTH_UNKNOWN = 0;
static constexpr uint8_t POWER_SUPPLY_HEALTH_GOOD = 1;
static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERHEAT = 2;
static constexpr uint8_t POWER_SUPPLY_HEALTH_DEAD = 3;
static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4;
static constexpr uint8_t POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5;
static constexpr uint8_t POWER_SUPPLY_HEALTH_COLD = 6;
static constexpr uint8_t POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7;
static constexpr uint8_t POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NIMH = 1;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LION = 2;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIPO = 3;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIFE = 4;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NICD = 5;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIMN = 6;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_TERNARY = 7;
static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_VRLA = 8;
std_msgs::msg::Header header{};
float voltage{};
float temperature{};
float current{};
float charge{};
float capacity{};
float design_capacity{};
float percentage{};
uint8_t power_supply_status{};
uint8_t power_supply_health{};
uint8_t power_supply_technology{};
bool present{};
std::vector<float> cell_voltage{};
std::vector<float> cell_temperature{};
std::string location{};
std::string serial_number{};
bool operator==(const BatteryState& other) const { return this->header == other.header && this->voltage == other.voltage && this->temperature == other.temperature && this->current == other.current && this->charge == other.charge && this->capacity == other.capacity && this->design_capacity == other.design_capacity && this->percentage == other.percentage && this->power_supply_status == other.power_supply_status && this->power_supply_health == other.power_supply_health && this->power_supply_technology == other.power_supply_technology && this->present == other.present && this->cell_voltage == other.cell_voltage && this->cell_temperature == other.cell_temperature && this->location == other.location && this->serial_number == other.serial_number; }
bool operator!=(const BatteryState& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/BatteryState";
};
}
#endif
#ifndef DIMOS_MESSAGE_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62_TYPE
#define DIMOS_MESSAGE_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62_TYPE
namespace sensor_msgs::msg {
struct RegionOfInterest {
uint32_t x_offset{};
uint32_t y_offset{};
uint32_t height{};
uint32_t width{};
bool do_rectify{};
bool operator==(const RegionOfInterest& other) const { return this->x_offset == other.x_offset && this->y_offset == other.y_offset && this->height == other.height && this->width == other.width && this->do_rectify == other.do_rectify; }
bool operator!=(const RegionOfInterest& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "sensor_msgs/msg/RegionOfInterest";
};
}
#endif
#ifndef DIMOS_MESSAGE_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5_TYPE
#define DIMOS_MESSAGE_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5_TYPE
namespace sensor_msgs::msg {
struct CameraInfo {
std_msgs::msg::Header header{};
uint32_t height{};
uint32_t width{};
std::string distortion_model{};
std::vector<double> d{};
std::array<double, 9> k{};
std::array<double, 9> r{};
std::array<double, 12> p{};
uint32_t binning_x{};
uint32_t binning_y{};
sensor_msgs::msg::RegionOfInterest roi{};
bool operator==(const CameraInfo& other) const { return this->header == other.header && this->height == other.height && this->width == other.width && this->distortion_model == other.distortion_model && this->d == other.d && this->k == other.k && this->r == other.r && this->p == other.p && this->binning_x == other.binning_x && this->binning_y == other.binning_y && this->roi == other.roi; }
bool operator!=(const CameraInfo& other) const { return !(*this == other); }
void validate() const {
header.validate();
roi.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/CameraInfo";
};
}
#endif
#ifndef DIMOS_MESSAGE_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6_TYPE
#define DIMOS_MESSAGE_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6_TYPE
namespace sensor_msgs::msg {
struct ChannelFloat32 {
std::string name{};
std::vector<float> values{};
bool operator==(const ChannelFloat32& other) const { return this->name == other.name && this->values == other.values; }
bool operator!=(const ChannelFloat32& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "sensor_msgs/msg/ChannelFloat32";
};
}
#endif
#ifndef DIMOS_MESSAGE_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A_TYPE
#define DIMOS_MESSAGE_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A_TYPE
namespace sensor_msgs::msg {
struct CompressedImage {
std_msgs::msg::Header header{};
std::string format{};
std::vector<uint8_t> data{};
bool operator==(const CompressedImage& other) const { return this->header == other.header && this->format == other.format && this->data == other.data; }
bool operator!=(const CompressedImage& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/CompressedImage";
};
}
#endif
#ifndef DIMOS_MESSAGE_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87_TYPE
#define DIMOS_MESSAGE_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87_TYPE
namespace sensor_msgs::msg {
struct FluidPressure {
std_msgs::msg::Header header{};
double fluid_pressure{};
double variance{};
bool operator==(const FluidPressure& other) const { return this->header == other.header && this->fluid_pressure == other.fluid_pressure && this->variance == other.variance; }
bool operator!=(const FluidPressure& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/FluidPressure";
};
}
#endif
#ifndef DIMOS_MESSAGE_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733_TYPE
#define DIMOS_MESSAGE_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733_TYPE
namespace sensor_msgs::msg {
struct Illuminance {
std_msgs::msg::Header header{};
double illuminance{};
double variance{};
bool operator==(const Illuminance& other) const { return this->header == other.header && this->illuminance == other.illuminance && this->variance == other.variance; }
bool operator!=(const Illuminance& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Illuminance";
};
}
#endif
#ifndef DIMOS_MESSAGE_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25_TYPE
#define DIMOS_MESSAGE_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25_TYPE
namespace sensor_msgs::msg {
struct Image {
std_msgs::msg::Header header{};
uint32_t height{};
uint32_t width{};
std::string encoding{};
uint8_t is_bigendian{};
uint32_t step{};
std::vector<uint8_t> data{};
bool operator==(const Image& other) const { return this->header == other.header && this->height == other.height && this->width == other.width && this->encoding == other.encoding && this->is_bigendian == other.is_bigendian && this->step == other.step && this->data == other.data; }
bool operator!=(const Image& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Image";
};
}
#endif
#ifndef DIMOS_MESSAGE_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75_TYPE
#define DIMOS_MESSAGE_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75_TYPE
namespace sensor_msgs::msg {
struct Imu {
std_msgs::msg::Header header{};
geometry_msgs::msg::Quaternion orientation{};
std::array<double, 9> orientation_covariance{};
geometry_msgs::msg::Vector3 angular_velocity{};
std::array<double, 9> angular_velocity_covariance{};
geometry_msgs::msg::Vector3 linear_acceleration{};
std::array<double, 9> linear_acceleration_covariance{};
bool operator==(const Imu& other) const { return this->header == other.header && this->orientation == other.orientation && this->orientation_covariance == other.orientation_covariance && this->angular_velocity == other.angular_velocity && this->angular_velocity_covariance == other.angular_velocity_covariance && this->linear_acceleration == other.linear_acceleration && this->linear_acceleration_covariance == other.linear_acceleration_covariance; }
bool operator!=(const Imu& other) const { return !(*this == other); }
void validate() const {
header.validate();
orientation.validate();
angular_velocity.validate();
linear_acceleration.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Imu";
};
}
#endif
#ifndef DIMOS_MESSAGE_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752_TYPE
#define DIMOS_MESSAGE_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752_TYPE
namespace sensor_msgs::msg {
struct JointState {
std_msgs::msg::Header header{};
std::vector<std::string> name{};
std::vector<double> position{};
std::vector<double> velocity{};
std::vector<double> effort{};
bool operator==(const JointState& other) const { return this->header == other.header && this->name == other.name && this->position == other.position && this->velocity == other.velocity && this->effort == other.effort; }
bool operator!=(const JointState& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/JointState";
};
}
#endif
#ifndef DIMOS_MESSAGE_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8_TYPE
#define DIMOS_MESSAGE_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8_TYPE
namespace sensor_msgs::msg {
struct Joy {
std_msgs::msg::Header header{};
std::vector<float> axes{};
std::vector<int32_t> buttons{};
bool operator==(const Joy& other) const { return this->header == other.header && this->axes == other.axes && this->buttons == other.buttons; }
bool operator!=(const Joy& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Joy";
};
}
#endif
#ifndef DIMOS_MESSAGE_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F_TYPE
#define DIMOS_MESSAGE_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F_TYPE
namespace sensor_msgs::msg {
struct JoyFeedback {
static constexpr uint8_t TYPE_LED = 0;
static constexpr uint8_t TYPE_RUMBLE = 1;
static constexpr uint8_t TYPE_BUZZER = 2;
uint8_t type{};
uint8_t id{};
float intensity{};
bool operator==(const JoyFeedback& other) const { return this->type == other.type && this->id == other.id && this->intensity == other.intensity; }
bool operator!=(const JoyFeedback& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "sensor_msgs/msg/JoyFeedback";
};
}
#endif
#ifndef DIMOS_MESSAGE_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF_TYPE
#define DIMOS_MESSAGE_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF_TYPE
namespace sensor_msgs::msg {
struct JoyFeedbackArray {
std::vector<sensor_msgs::msg::JoyFeedback> array{};
bool operator==(const JoyFeedbackArray& other) const { return this->array == other.array; }
bool operator!=(const JoyFeedbackArray& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : array) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/JoyFeedbackArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71_TYPE
#define DIMOS_MESSAGE_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71_TYPE
namespace sensor_msgs::msg {
struct LaserEcho {
std::vector<float> echoes{};
bool operator==(const LaserEcho& other) const { return this->echoes == other.echoes; }
bool operator!=(const LaserEcho& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "sensor_msgs/msg/LaserEcho";
};
}
#endif
#ifndef DIMOS_MESSAGE_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419_TYPE
#define DIMOS_MESSAGE_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419_TYPE
namespace sensor_msgs::msg {
struct LaserScan {
std_msgs::msg::Header header{};
float angle_min{};
float angle_max{};
float angle_increment{};
float time_increment{};
float scan_time{};
float range_min{};
float range_max{};
std::vector<float> ranges{};
std::vector<float> intensities{};
bool operator==(const LaserScan& other) const { return this->header == other.header && this->angle_min == other.angle_min && this->angle_max == other.angle_max && this->angle_increment == other.angle_increment && this->time_increment == other.time_increment && this->scan_time == other.scan_time && this->range_min == other.range_min && this->range_max == other.range_max && this->ranges == other.ranges && this->intensities == other.intensities; }
bool operator!=(const LaserScan& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/LaserScan";
};
}
#endif
#ifndef DIMOS_MESSAGE_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E_TYPE
#define DIMOS_MESSAGE_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E_TYPE
namespace sensor_msgs::msg {
struct MagneticField {
std_msgs::msg::Header header{};
geometry_msgs::msg::Vector3 magnetic_field{};
std::array<double, 9> magnetic_field_covariance{};
bool operator==(const MagneticField& other) const { return this->header == other.header && this->magnetic_field == other.magnetic_field && this->magnetic_field_covariance == other.magnetic_field_covariance; }
bool operator!=(const MagneticField& other) const { return !(*this == other); }
void validate() const {
header.validate();
magnetic_field.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/MagneticField";
};
}
#endif
#ifndef DIMOS_MESSAGE_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C_TYPE
#define DIMOS_MESSAGE_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C_TYPE
namespace sensor_msgs::msg {
struct MultiDOFJointState {
std_msgs::msg::Header header{};
std::vector<std::string> joint_names{};
std::vector<geometry_msgs::msg::Transform> transforms{};
std::vector<geometry_msgs::msg::Twist> twist{};
std::vector<geometry_msgs::msg::Wrench> wrench{};
bool operator==(const MultiDOFJointState& other) const { return this->header == other.header && this->joint_names == other.joint_names && this->transforms == other.transforms && this->twist == other.twist && this->wrench == other.wrench; }
bool operator!=(const MultiDOFJointState& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : transforms) { item.validate(); }
for (const auto& item : twist) { item.validate(); }
for (const auto& item : wrench) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/MultiDOFJointState";
};
}
#endif
#ifndef DIMOS_MESSAGE_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF_TYPE
#define DIMOS_MESSAGE_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF_TYPE
namespace sensor_msgs::msg {
struct MultiEchoLaserScan {
std_msgs::msg::Header header{};
float angle_min{};
float angle_max{};
float angle_increment{};
float time_increment{};
float scan_time{};
float range_min{};
float range_max{};
std::vector<sensor_msgs::msg::LaserEcho> ranges{};
std::vector<sensor_msgs::msg::LaserEcho> intensities{};
bool operator==(const MultiEchoLaserScan& other) const { return this->header == other.header && this->angle_min == other.angle_min && this->angle_max == other.angle_max && this->angle_increment == other.angle_increment && this->time_increment == other.time_increment && this->scan_time == other.scan_time && this->range_min == other.range_min && this->range_max == other.range_max && this->ranges == other.ranges && this->intensities == other.intensities; }
bool operator!=(const MultiEchoLaserScan& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : ranges) { item.validate(); }
for (const auto& item : intensities) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/MultiEchoLaserScan";
};
}
#endif
#ifndef DIMOS_MESSAGE_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2_TYPE
#define DIMOS_MESSAGE_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2_TYPE
namespace sensor_msgs::msg {
struct NavSatStatus {
static constexpr int8_t STATUS_UNKNOWN = -2;
static constexpr int8_t STATUS_NO_FIX = -1;
static constexpr int8_t STATUS_FIX = 0;
static constexpr int8_t STATUS_SBAS_FIX = 1;
static constexpr int8_t STATUS_GBAS_FIX = 2;
static constexpr uint16_t SERVICE_UNKNOWN = 0;
static constexpr uint16_t SERVICE_GPS = 1;
static constexpr uint16_t SERVICE_GLONASS = 2;
static constexpr uint16_t SERVICE_COMPASS = 4;
static constexpr uint16_t SERVICE_GALILEO = 8;
int8_t status{-2};
uint16_t service{};
bool operator==(const NavSatStatus& other) const { return this->status == other.status && this->service == other.service; }
bool operator!=(const NavSatStatus& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "sensor_msgs/msg/NavSatStatus";
};
}
#endif
#ifndef DIMOS_MESSAGE_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9_TYPE
#define DIMOS_MESSAGE_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9_TYPE
namespace sensor_msgs::msg {
struct NavSatFix {
static constexpr uint8_t COVARIANCE_TYPE_UNKNOWN = 0;
static constexpr uint8_t COVARIANCE_TYPE_APPROXIMATED = 1;
static constexpr uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
static constexpr uint8_t COVARIANCE_TYPE_KNOWN = 3;
std_msgs::msg::Header header{};
sensor_msgs::msg::NavSatStatus status{};
double latitude{};
double longitude{};
double altitude{};
std::array<double, 9> position_covariance{};
uint8_t position_covariance_type{};
bool operator==(const NavSatFix& other) const { return this->header == other.header && this->status == other.status && this->latitude == other.latitude && this->longitude == other.longitude && this->altitude == other.altitude && this->position_covariance == other.position_covariance && this->position_covariance_type == other.position_covariance_type; }
bool operator!=(const NavSatFix& other) const { return !(*this == other); }
void validate() const {
header.validate();
status.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/NavSatFix";
};
}
#endif
#ifndef DIMOS_MESSAGE_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D_TYPE
#define DIMOS_MESSAGE_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D_TYPE
namespace sensor_msgs::msg {
struct PointCloud {
std_msgs::msg::Header header{};
std::vector<geometry_msgs::msg::Point32> points{};
std::vector<sensor_msgs::msg::ChannelFloat32> channels{};
bool operator==(const PointCloud& other) const { return this->header == other.header && this->points == other.points && this->channels == other.channels; }
bool operator!=(const PointCloud& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : points) { item.validate(); }
for (const auto& item : channels) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/PointCloud";
};
}
#endif
#ifndef DIMOS_MESSAGE_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018_TYPE
#define DIMOS_MESSAGE_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018_TYPE
namespace sensor_msgs::msg {
struct PointField {
static constexpr uint8_t INT8 = 1;
static constexpr uint8_t UINT8 = 2;
static constexpr uint8_t INT16 = 3;
static constexpr uint8_t UINT16 = 4;
static constexpr uint8_t INT32 = 5;
static constexpr uint8_t UINT32 = 6;
static constexpr uint8_t FLOAT32 = 7;
static constexpr uint8_t FLOAT64 = 8;
std::string name{};
uint32_t offset{};
uint8_t datatype{};
uint32_t count{};
bool operator==(const PointField& other) const { return this->name == other.name && this->offset == other.offset && this->datatype == other.datatype && this->count == other.count; }
bool operator!=(const PointField& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "sensor_msgs/msg/PointField";
};
}
#endif
#ifndef DIMOS_MESSAGE_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973_TYPE
#define DIMOS_MESSAGE_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973_TYPE
namespace sensor_msgs::msg {
struct PointCloud2 {
std_msgs::msg::Header header{};
uint32_t height{};
uint32_t width{};
std::vector<sensor_msgs::msg::PointField> fields{};
bool is_bigendian{};
uint32_t point_step{};
uint32_t row_step{};
std::vector<uint8_t> data{};
bool is_dense{};
bool operator==(const PointCloud2& other) const { return this->header == other.header && this->height == other.height && this->width == other.width && this->fields == other.fields && this->is_bigendian == other.is_bigendian && this->point_step == other.point_step && this->row_step == other.row_step && this->data == other.data && this->is_dense == other.is_dense; }
bool operator!=(const PointCloud2& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : fields) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/PointCloud2";
};
}
#endif
#ifndef DIMOS_MESSAGE_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579_TYPE
#define DIMOS_MESSAGE_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579_TYPE
namespace sensor_msgs::msg {
struct Range {
static constexpr uint8_t ULTRASOUND = 0;
static constexpr uint8_t INFRARED = 1;
std_msgs::msg::Header header{};
uint8_t radiation_type{};
float field_of_view{};
float min_range{};
float max_range{};
float range{};
float variance{};
bool operator==(const Range& other) const { return this->header == other.header && this->radiation_type == other.radiation_type && this->field_of_view == other.field_of_view && this->min_range == other.min_range && this->max_range == other.max_range && this->range == other.range && this->variance == other.variance; }
bool operator!=(const Range& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Range";
};
}
#endif
#ifndef DIMOS_MESSAGE_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0_TYPE
#define DIMOS_MESSAGE_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0_TYPE
namespace sensor_msgs::msg {
struct RelativeHumidity {
std_msgs::msg::Header header{};
double relative_humidity{};
double variance{};
bool operator==(const RelativeHumidity& other) const { return this->header == other.header && this->relative_humidity == other.relative_humidity && this->variance == other.variance; }
bool operator!=(const RelativeHumidity& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/RelativeHumidity";
};
}
#endif
#ifndef DIMOS_MESSAGE_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D_TYPE
#define DIMOS_MESSAGE_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D_TYPE
namespace sensor_msgs::msg {
struct Temperature {
std_msgs::msg::Header header{};
double temperature{};
double variance{};
bool operator==(const Temperature& other) const { return this->header == other.header && this->temperature == other.temperature && this->variance == other.variance; }
bool operator!=(const Temperature& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Temperature";
};
}
#endif
#ifndef DIMOS_MESSAGE_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430_TYPE
#define DIMOS_MESSAGE_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430_TYPE
namespace sensor_msgs::msg {
struct TimeReference {
std_msgs::msg::Header header{};
builtin_interfaces::msg::Time time_ref{};
std::string source{};
bool operator==(const TimeReference& other) const { return this->header == other.header && this->time_ref == other.time_ref && this->source == other.source; }
bool operator!=(const TimeReference& other) const { return !(*this == other); }
void validate() const {
header.validate();
time_ref.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/TimeReference";
};
}
#endif
#ifndef DIMOS_MESSAGE_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50_TYPE
#define DIMOS_MESSAGE_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50_TYPE
namespace shape_msgs::msg {
struct MeshTriangle {
std::array<uint32_t, 3> vertex_indices{};
bool operator==(const MeshTriangle& other) const { return this->vertex_indices == other.vertex_indices; }
bool operator!=(const MeshTriangle& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "shape_msgs/msg/MeshTriangle";
};
}
#endif
#ifndef DIMOS_MESSAGE_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68_TYPE
#define DIMOS_MESSAGE_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68_TYPE
namespace shape_msgs::msg {
struct Mesh {
std::vector<shape_msgs::msg::MeshTriangle> triangles{};
std::vector<geometry_msgs::msg::Point> vertices{};
bool operator==(const Mesh& other) const { return this->triangles == other.triangles && this->vertices == other.vertices; }
bool operator!=(const Mesh& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : triangles) { item.validate(); }
for (const auto& item : vertices) { item.validate(); }
}
static constexpr const char* msg_name = "shape_msgs/msg/Mesh";
};
}
#endif
#ifndef DIMOS_MESSAGE_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5_TYPE
#define DIMOS_MESSAGE_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5_TYPE
namespace shape_msgs::msg {
struct Plane {
std::array<double, 4> coef{};
bool operator==(const Plane& other) const { return this->coef == other.coef; }
bool operator!=(const Plane& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "shape_msgs/msg/Plane";
};
}
#endif
#ifndef DIMOS_MESSAGE_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5_TYPE
#define DIMOS_MESSAGE_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5_TYPE
namespace shape_msgs::msg {
struct SolidPrimitive {
static constexpr uint8_t BOX = 1;
static constexpr uint8_t SPHERE = 2;
static constexpr uint8_t CYLINDER = 3;
static constexpr uint8_t CONE = 4;
static constexpr uint8_t PRISM = 5;
static constexpr uint8_t BOX_X = 0;
static constexpr uint8_t BOX_Y = 1;
static constexpr uint8_t BOX_Z = 2;
static constexpr uint8_t SPHERE_RADIUS = 0;
static constexpr uint8_t CYLINDER_HEIGHT = 0;
static constexpr uint8_t CYLINDER_RADIUS = 1;
static constexpr uint8_t CONE_HEIGHT = 0;
static constexpr uint8_t CONE_RADIUS = 1;
static constexpr uint8_t PRISM_HEIGHT = 0;
uint8_t type{};
std::vector<double> dimensions{};
geometry_msgs::msg::Polygon polygon{};
bool operator==(const SolidPrimitive& other) const { return this->type == other.type && this->dimensions == other.dimensions && this->polygon == other.polygon; }
bool operator!=(const SolidPrimitive& other) const { return !(*this == other); }
void validate() const {
if (dimensions.size() > 3) throw std::length_error("dimensions exceeds sequence bound");
polygon.validate();
}
static constexpr const char* msg_name = "shape_msgs/msg/SolidPrimitive";
};
}
#endif
#ifndef DIMOS_MESSAGE_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B_TYPE
#define DIMOS_MESSAGE_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B_TYPE
namespace std_msgs::msg {
struct Bool {
bool data{};
bool operator==(const Bool& other) const { return this->data == other.data; }
bool operator!=(const Bool& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Bool";
};
}
#endif
#ifndef DIMOS_MESSAGE_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141_TYPE
#define DIMOS_MESSAGE_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141_TYPE
namespace std_msgs::msg {
struct Byte {
uint8_t data{};
bool operator==(const Byte& other) const { return this->data == other.data; }
bool operator!=(const Byte& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Byte";
};
}
#endif
#ifndef DIMOS_MESSAGE_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2_TYPE
#define DIMOS_MESSAGE_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2_TYPE
namespace std_msgs::msg {
struct MultiArrayDimension {
std::string label{};
uint32_t size{};
uint32_t stride{};
bool operator==(const MultiArrayDimension& other) const { return this->label == other.label && this->size == other.size && this->stride == other.stride; }
bool operator!=(const MultiArrayDimension& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/MultiArrayDimension";
};
}
#endif
#ifndef DIMOS_MESSAGE_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8_TYPE
#define DIMOS_MESSAGE_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8_TYPE
namespace std_msgs::msg {
struct MultiArrayLayout {
std::vector<std_msgs::msg::MultiArrayDimension> dim{};
uint32_t data_offset{};
bool operator==(const MultiArrayLayout& other) const { return this->dim == other.dim && this->data_offset == other.data_offset; }
bool operator!=(const MultiArrayLayout& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : dim) { item.validate(); }
}
static constexpr const char* msg_name = "std_msgs/msg/MultiArrayLayout";
};
}
#endif
#ifndef DIMOS_MESSAGE_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39_TYPE
#define DIMOS_MESSAGE_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39_TYPE
namespace std_msgs::msg {
struct ByteMultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<uint8_t> data{};
bool operator==(const ByteMultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const ByteMultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/ByteMultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7_TYPE
#define DIMOS_MESSAGE_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7_TYPE
namespace std_msgs::msg {
struct Char {
uint8_t data{};
bool operator==(const Char& other) const { return this->data == other.data; }
bool operator!=(const Char& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Char";
};
}
#endif
#ifndef DIMOS_MESSAGE_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F_TYPE
#define DIMOS_MESSAGE_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F_TYPE
namespace std_msgs::msg {
struct ColorRGBA {
float r{};
float g{};
float b{};
float a{};
bool operator==(const ColorRGBA& other) const { return this->r == other.r && this->g == other.g && this->b == other.b && this->a == other.a; }
bool operator!=(const ColorRGBA& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/ColorRGBA";
};
}
#endif
#ifndef DIMOS_MESSAGE_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380_TYPE
#define DIMOS_MESSAGE_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380_TYPE
namespace std_msgs::msg {
struct Empty {
bool operator==(const Empty&) const { return true; }
bool operator!=(const Empty& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Empty";
};
}
#endif
#ifndef DIMOS_MESSAGE_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC_TYPE
#define DIMOS_MESSAGE_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC_TYPE
namespace std_msgs::msg {
struct Float32 {
float data{};
bool operator==(const Float32& other) const { return this->data == other.data; }
bool operator!=(const Float32& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Float32";
};
}
#endif
#ifndef DIMOS_MESSAGE_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1_TYPE
#define DIMOS_MESSAGE_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1_TYPE
namespace std_msgs::msg {
struct Float32MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<float> data{};
bool operator==(const Float32MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const Float32MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Float32MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33_TYPE
#define DIMOS_MESSAGE_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33_TYPE
namespace std_msgs::msg {
struct Float64 {
double data{};
bool operator==(const Float64& other) const { return this->data == other.data; }
bool operator!=(const Float64& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Float64";
};
}
#endif
#ifndef DIMOS_MESSAGE_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330_TYPE
#define DIMOS_MESSAGE_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330_TYPE
namespace std_msgs::msg {
struct Float64MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<double> data{};
bool operator==(const Float64MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const Float64MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Float64MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731_TYPE
#define DIMOS_MESSAGE_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731_TYPE
namespace std_msgs::msg {
struct Int16 {
int16_t data{};
bool operator==(const Int16& other) const { return this->data == other.data; }
bool operator!=(const Int16& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Int16";
};
}
#endif
#ifndef DIMOS_MESSAGE_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119_TYPE
#define DIMOS_MESSAGE_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119_TYPE
namespace std_msgs::msg {
struct Int16MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<int16_t> data{};
bool operator==(const Int16MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const Int16MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Int16MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488_TYPE
#define DIMOS_MESSAGE_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488_TYPE
namespace std_msgs::msg {
struct Int32 {
int32_t data{};
bool operator==(const Int32& other) const { return this->data == other.data; }
bool operator!=(const Int32& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Int32";
};
}
#endif
#ifndef DIMOS_MESSAGE_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969_TYPE
#define DIMOS_MESSAGE_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969_TYPE
namespace std_msgs::msg {
struct Int32MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<int32_t> data{};
bool operator==(const Int32MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const Int32MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Int32MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D_TYPE
#define DIMOS_MESSAGE_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D_TYPE
namespace std_msgs::msg {
struct Int64 {
int64_t data{};
bool operator==(const Int64& other) const { return this->data == other.data; }
bool operator!=(const Int64& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Int64";
};
}
#endif
#ifndef DIMOS_MESSAGE_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F_TYPE
#define DIMOS_MESSAGE_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F_TYPE
namespace std_msgs::msg {
struct Int64MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<int64_t> data{};
bool operator==(const Int64MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const Int64MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Int64MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575_TYPE
#define DIMOS_MESSAGE_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575_TYPE
namespace std_msgs::msg {
struct Int8 {
int8_t data{};
bool operator==(const Int8& other) const { return this->data == other.data; }
bool operator!=(const Int8& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/Int8";
};
}
#endif
#ifndef DIMOS_MESSAGE_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C_TYPE
#define DIMOS_MESSAGE_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C_TYPE
namespace std_msgs::msg {
struct Int8MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<int8_t> data{};
bool operator==(const Int8MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const Int8MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Int8MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1_TYPE
#define DIMOS_MESSAGE_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1_TYPE
namespace std_msgs::msg {
struct String {
std::string data{};
bool operator==(const String& other) const { return this->data == other.data; }
bool operator!=(const String& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/String";
};
}
#endif
#ifndef DIMOS_MESSAGE_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5_TYPE
#define DIMOS_MESSAGE_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5_TYPE
namespace std_msgs::msg {
struct UInt16 {
uint16_t data{};
bool operator==(const UInt16& other) const { return this->data == other.data; }
bool operator!=(const UInt16& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/UInt16";
};
}
#endif
#ifndef DIMOS_MESSAGE_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81_TYPE
#define DIMOS_MESSAGE_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81_TYPE
namespace std_msgs::msg {
struct UInt16MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<uint16_t> data{};
bool operator==(const UInt16MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const UInt16MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/UInt16MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F_TYPE
#define DIMOS_MESSAGE_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F_TYPE
namespace std_msgs::msg {
struct UInt32 {
uint32_t data{};
bool operator==(const UInt32& other) const { return this->data == other.data; }
bool operator!=(const UInt32& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/UInt32";
};
}
#endif
#ifndef DIMOS_MESSAGE_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD_TYPE
#define DIMOS_MESSAGE_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD_TYPE
namespace std_msgs::msg {
struct UInt32MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<uint32_t> data{};
bool operator==(const UInt32MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const UInt32MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/UInt32MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7_TYPE
#define DIMOS_MESSAGE_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7_TYPE
namespace std_msgs::msg {
struct UInt64 {
uint64_t data{};
bool operator==(const UInt64& other) const { return this->data == other.data; }
bool operator!=(const UInt64& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/UInt64";
};
}
#endif
#ifndef DIMOS_MESSAGE_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95_TYPE
#define DIMOS_MESSAGE_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95_TYPE
namespace std_msgs::msg {
struct UInt64MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<uint64_t> data{};
bool operator==(const UInt64MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const UInt64MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/UInt64MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4_TYPE
#define DIMOS_MESSAGE_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4_TYPE
namespace std_msgs::msg {
struct UInt8 {
uint8_t data{};
bool operator==(const UInt8& other) const { return this->data == other.data; }
bool operator!=(const UInt8& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "std_msgs/msg/UInt8";
};
}
#endif
#ifndef DIMOS_MESSAGE_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636_TYPE
#define DIMOS_MESSAGE_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636_TYPE
namespace std_msgs::msg {
struct UInt8MultiArray {
std_msgs::msg::MultiArrayLayout layout{};
std::vector<uint8_t> data{};
bool operator==(const UInt8MultiArray& other) const { return this->layout == other.layout && this->data == other.data; }
bool operator!=(const UInt8MultiArray& other) const { return !(*this == other); }
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/UInt8MultiArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C_TYPE
#define DIMOS_MESSAGE_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C_TYPE
namespace tf2_msgs::msg {
struct TF2Error {
static constexpr uint8_t NO_ERROR = 0;
static constexpr uint8_t LOOKUP_ERROR = 1;
static constexpr uint8_t CONNECTIVITY_ERROR = 2;
static constexpr uint8_t EXTRAPOLATION_ERROR = 3;
static constexpr uint8_t INVALID_ARGUMENT_ERROR = 4;
static constexpr uint8_t TIMEOUT_ERROR = 5;
static constexpr uint8_t TRANSFORM_ERROR = 6;
uint8_t error{};
std::string error_string{};
bool operator==(const TF2Error& other) const { return this->error == other.error && this->error_string == other.error_string; }
bool operator!=(const TF2Error& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "tf2_msgs/msg/TF2Error";
};
}
#endif
#ifndef DIMOS_MESSAGE_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A_TYPE
#define DIMOS_MESSAGE_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A_TYPE
namespace tf2_msgs::msg {
struct TFMessage {
std::vector<geometry_msgs::msg::TransformStamped> transforms{};
bool operator==(const TFMessage& other) const { return this->transforms == other.transforms; }
bool operator!=(const TFMessage& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : transforms) { item.validate(); }
}
static constexpr const char* msg_name = "tf2_msgs/msg/TFMessage";
};
}
#endif
#ifndef DIMOS_MESSAGE_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848_TYPE
#define DIMOS_MESSAGE_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848_TYPE
namespace trajectory_msgs::msg {
struct JointTrajectoryPoint {
std::vector<double> positions{};
std::vector<double> velocities{};
std::vector<double> accelerations{};
std::vector<double> effort{};
builtin_interfaces::msg::Duration time_from_start{};
bool operator==(const JointTrajectoryPoint& other) const { return this->positions == other.positions && this->velocities == other.velocities && this->accelerations == other.accelerations && this->effort == other.effort && this->time_from_start == other.time_from_start; }
bool operator!=(const JointTrajectoryPoint& other) const { return !(*this == other); }
void validate() const {
time_from_start.validate();
}
static constexpr const char* msg_name = "trajectory_msgs/msg/JointTrajectoryPoint";
};
}
#endif
#ifndef DIMOS_MESSAGE_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56_TYPE
#define DIMOS_MESSAGE_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56_TYPE
namespace trajectory_msgs::msg {
struct JointTrajectory {
std_msgs::msg::Header header{};
std::vector<std::string> joint_names{};
std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points{};
bool operator==(const JointTrajectory& other) const { return this->header == other.header && this->joint_names == other.joint_names && this->points == other.points; }
bool operator!=(const JointTrajectory& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : points) { item.validate(); }
}
static constexpr const char* msg_name = "trajectory_msgs/msg/JointTrajectory";
};
}
#endif
#ifndef DIMOS_MESSAGE_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA_TYPE
#define DIMOS_MESSAGE_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA_TYPE
namespace trajectory_msgs::msg {
struct MultiDOFJointTrajectoryPoint {
std::vector<geometry_msgs::msg::Transform> transforms{};
std::vector<geometry_msgs::msg::Twist> velocities{};
std::vector<geometry_msgs::msg::Twist> accelerations{};
builtin_interfaces::msg::Duration time_from_start{};
bool operator==(const MultiDOFJointTrajectoryPoint& other) const { return this->transforms == other.transforms && this->velocities == other.velocities && this->accelerations == other.accelerations && this->time_from_start == other.time_from_start; }
bool operator!=(const MultiDOFJointTrajectoryPoint& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : transforms) { item.validate(); }
for (const auto& item : velocities) { item.validate(); }
for (const auto& item : accelerations) { item.validate(); }
time_from_start.validate();
}
static constexpr const char* msg_name = "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint";
};
}
#endif
#ifndef DIMOS_MESSAGE_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2_TYPE
#define DIMOS_MESSAGE_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2_TYPE
namespace trajectory_msgs::msg {
struct MultiDOFJointTrajectory {
std_msgs::msg::Header header{};
std::vector<std::string> joint_names{};
std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint> points{};
bool operator==(const MultiDOFJointTrajectory& other) const { return this->header == other.header && this->joint_names == other.joint_names && this->points == other.points; }
bool operator!=(const MultiDOFJointTrajectory& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : points) { item.validate(); }
}
static constexpr const char* msg_name = "trajectory_msgs/msg/MultiDOFJointTrajectory";
};
}
#endif
#ifndef DIMOS_MESSAGE_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E_TYPE
#define DIMOS_MESSAGE_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E_TYPE
namespace vision_msgs::msg {
struct BoundingBox2DArray {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::BoundingBox2D> boxes{};
bool operator==(const BoundingBox2DArray& other) const { return this->header == other.header && this->boxes == other.boxes; }
bool operator!=(const BoundingBox2DArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : boxes) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/BoundingBox2DArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65_TYPE
#define DIMOS_MESSAGE_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65_TYPE
namespace vision_msgs::msg {
struct BoundingBox3DArray {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::BoundingBox3D> boxes{};
bool operator==(const BoundingBox3DArray& other) const { return this->header == other.header && this->boxes == other.boxes; }
bool operator!=(const BoundingBox3DArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : boxes) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/BoundingBox3DArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A_TYPE
#define DIMOS_MESSAGE_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A_TYPE
namespace vision_msgs::msg {
struct ObjectHypothesis {
std::string class_id{};
double score{};
bool operator==(const ObjectHypothesis& other) const { return this->class_id == other.class_id && this->score == other.score; }
bool operator!=(const ObjectHypothesis& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "vision_msgs/msg/ObjectHypothesis";
};
}
#endif
#ifndef DIMOS_MESSAGE_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339_TYPE
#define DIMOS_MESSAGE_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339_TYPE
namespace vision_msgs::msg {
struct Classification {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::ObjectHypothesis> results{};
bool operator==(const Classification& other) const { return this->header == other.header && this->results == other.results; }
bool operator!=(const Classification& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : results) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/Classification";
};
}
#endif
#ifndef DIMOS_MESSAGE_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1_TYPE
#define DIMOS_MESSAGE_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1_TYPE
namespace vision_msgs::msg {
struct ObjectHypothesisWithPose {
vision_msgs::msg::ObjectHypothesis hypothesis{};
geometry_msgs::msg::PoseWithCovariance pose{};
bool operator==(const ObjectHypothesisWithPose& other) const { return this->hypothesis == other.hypothesis && this->pose == other.pose; }
bool operator!=(const ObjectHypothesisWithPose& other) const { return !(*this == other); }
void validate() const {
hypothesis.validate();
pose.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/ObjectHypothesisWithPose";
};
}
#endif
#ifndef DIMOS_MESSAGE_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29_TYPE
#define DIMOS_MESSAGE_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29_TYPE
namespace vision_msgs::msg {
struct Detection2D {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::ObjectHypothesisWithPose> results{};
vision_msgs::msg::BoundingBox2D bbox{};
std::string id{};
bool operator==(const Detection2D& other) const { return this->header == other.header && this->results == other.results && this->bbox == other.bbox && this->id == other.id; }
bool operator!=(const Detection2D& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : results) { item.validate(); }
bbox.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/Detection2D";
};
}
#endif
#ifndef DIMOS_MESSAGE_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A_TYPE
#define DIMOS_MESSAGE_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A_TYPE
namespace vision_msgs::msg {
struct Detection2DArray {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::Detection2D> detections{};
bool operator==(const Detection2DArray& other) const { return this->header == other.header && this->detections == other.detections; }
bool operator!=(const Detection2DArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : detections) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/Detection2DArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C_TYPE
#define DIMOS_MESSAGE_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C_TYPE
namespace vision_msgs::msg {
struct Detection3D {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::ObjectHypothesisWithPose> results{};
vision_msgs::msg::BoundingBox3D bbox{};
std::string id{};
bool operator==(const Detection3D& other) const { return this->header == other.header && this->results == other.results && this->bbox == other.bbox && this->id == other.id; }
bool operator!=(const Detection3D& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : results) { item.validate(); }
bbox.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/Detection3D";
};
}
#endif
#ifndef DIMOS_MESSAGE_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE_TYPE
#define DIMOS_MESSAGE_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE_TYPE
namespace vision_msgs::msg {
struct Detection3DArray {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::Detection3D> detections{};
bool operator==(const Detection3DArray& other) const { return this->header == other.header && this->detections == other.detections; }
bool operator!=(const Detection3DArray& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : detections) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/Detection3DArray";
};
}
#endif
#ifndef DIMOS_MESSAGE_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A_TYPE
#define DIMOS_MESSAGE_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A_TYPE
namespace vision_msgs::msg {
struct VisionClass {
uint16_t class_id{};
std::string class_name{};
bool operator==(const VisionClass& other) const { return this->class_id == other.class_id && this->class_name == other.class_name; }
bool operator!=(const VisionClass& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "vision_msgs/msg/VisionClass";
};
}
#endif
#ifndef DIMOS_MESSAGE_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062_TYPE
#define DIMOS_MESSAGE_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062_TYPE
namespace vision_msgs::msg {
struct LabelInfo {
std_msgs::msg::Header header{};
std::vector<vision_msgs::msg::VisionClass> class_map{};
float threshold{};
bool operator==(const LabelInfo& other) const { return this->header == other.header && this->class_map == other.class_map && this->threshold == other.threshold; }
bool operator!=(const LabelInfo& other) const { return !(*this == other); }
void validate() const {
header.validate();
for (const auto& item : class_map) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/LabelInfo";
};
}
#endif
#ifndef DIMOS_MESSAGE_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2_TYPE
#define DIMOS_MESSAGE_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2_TYPE
namespace vision_msgs::msg {
struct VisionInfo {
std_msgs::msg::Header header{};
std::string method{};
std::string database_location{};
int32_t database_version{};
bool operator==(const VisionInfo& other) const { return this->header == other.header && this->method == other.method && this->database_location == other.database_location && this->database_version == other.database_version; }
bool operator!=(const VisionInfo& other) const { return !(*this == other); }
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/VisionInfo";
};
}
#endif
#ifndef DIMOS_MESSAGE_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC_TYPE
#define DIMOS_MESSAGE_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC_TYPE
namespace visualization_msgs::msg {
struct ImageMarker {
static constexpr int32_t CIRCLE = 0;
static constexpr int32_t LINE_STRIP = 1;
static constexpr int32_t LINE_LIST = 2;
static constexpr int32_t POLYGON = 3;
static constexpr int32_t POINTS = 4;
static constexpr int32_t ADD = 0;
static constexpr int32_t REMOVE = 1;
std_msgs::msg::Header header{};
std::string ns{};
int32_t id{};
int32_t type{};
int32_t action{};
geometry_msgs::msg::Point position{};
float scale{};
std_msgs::msg::ColorRGBA outline_color{};
uint8_t filled{};
std_msgs::msg::ColorRGBA fill_color{};
builtin_interfaces::msg::Duration lifetime{};
std::vector<geometry_msgs::msg::Point> points{};
std::vector<std_msgs::msg::ColorRGBA> outline_colors{};
bool operator==(const ImageMarker& other) const { return this->header == other.header && this->ns == other.ns && this->id == other.id && this->type == other.type && this->action == other.action && this->position == other.position && this->scale == other.scale && this->outline_color == other.outline_color && this->filled == other.filled && this->fill_color == other.fill_color && this->lifetime == other.lifetime && this->points == other.points && this->outline_colors == other.outline_colors; }
bool operator!=(const ImageMarker& other) const { return !(*this == other); }
void validate() const {
header.validate();
position.validate();
outline_color.validate();
fill_color.validate();
lifetime.validate();
for (const auto& item : points) { item.validate(); }
for (const auto& item : outline_colors) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/ImageMarker";
};
}
#endif
#ifndef DIMOS_MESSAGE_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264_TYPE
#define DIMOS_MESSAGE_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264_TYPE
namespace visualization_msgs::msg {
struct MeshFile {
std::string filename{};
std::vector<uint8_t> data{};
bool operator==(const MeshFile& other) const { return this->filename == other.filename && this->data == other.data; }
bool operator!=(const MeshFile& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "visualization_msgs/msg/MeshFile";
};
}
#endif
#ifndef DIMOS_MESSAGE_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F_TYPE
#define DIMOS_MESSAGE_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F_TYPE
namespace visualization_msgs::msg {
struct UVCoordinate {
float u{};
float v{};
bool operator==(const UVCoordinate& other) const { return this->u == other.u && this->v == other.v; }
bool operator!=(const UVCoordinate& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "visualization_msgs/msg/UVCoordinate";
};
}
#endif
#ifndef DIMOS_MESSAGE_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF_TYPE
#define DIMOS_MESSAGE_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF_TYPE
namespace visualization_msgs::msg {
struct Marker {
static constexpr int32_t ARROW = 0;
static constexpr int32_t CUBE = 1;
static constexpr int32_t SPHERE = 2;
static constexpr int32_t CYLINDER = 3;
static constexpr int32_t LINE_STRIP = 4;
static constexpr int32_t LINE_LIST = 5;
static constexpr int32_t CUBE_LIST = 6;
static constexpr int32_t SPHERE_LIST = 7;
static constexpr int32_t POINTS = 8;
static constexpr int32_t TEXT_VIEW_FACING = 9;
static constexpr int32_t MESH_RESOURCE = 10;
static constexpr int32_t TRIANGLE_LIST = 11;
static constexpr int32_t ARROW_STRIP = 12;
static constexpr int32_t ADD = 0;
static constexpr int32_t MODIFY = 0;
static constexpr int32_t DELETE = 2;
static constexpr int32_t DELETEALL = 3;
std_msgs::msg::Header header{};
std::string ns{};
int32_t id{};
int32_t type{};
int32_t action{};
geometry_msgs::msg::Pose pose{};
geometry_msgs::msg::Vector3 scale{};
std_msgs::msg::ColorRGBA color{};
builtin_interfaces::msg::Duration lifetime{};
bool frame_locked{};
std::vector<geometry_msgs::msg::Point> points{};
std::vector<std_msgs::msg::ColorRGBA> colors{};
std::string texture_resource{};
sensor_msgs::msg::CompressedImage texture{};
std::vector<visualization_msgs::msg::UVCoordinate> uv_coordinates{};
std::string text{};
std::string mesh_resource{};
visualization_msgs::msg::MeshFile mesh_file{};
bool mesh_use_embedded_materials{};
bool operator==(const Marker& other) const { return this->header == other.header && this->ns == other.ns && this->id == other.id && this->type == other.type && this->action == other.action && this->pose == other.pose && this->scale == other.scale && this->color == other.color && this->lifetime == other.lifetime && this->frame_locked == other.frame_locked && this->points == other.points && this->colors == other.colors && this->texture_resource == other.texture_resource && this->texture == other.texture && this->uv_coordinates == other.uv_coordinates && this->text == other.text && this->mesh_resource == other.mesh_resource && this->mesh_file == other.mesh_file && this->mesh_use_embedded_materials == other.mesh_use_embedded_materials; }
bool operator!=(const Marker& other) const { return !(*this == other); }
void validate() const {
header.validate();
pose.validate();
scale.validate();
color.validate();
lifetime.validate();
for (const auto& item : points) { item.validate(); }
for (const auto& item : colors) { item.validate(); }
texture.validate();
for (const auto& item : uv_coordinates) { item.validate(); }
mesh_file.validate();
}
static constexpr const char* msg_name = "visualization_msgs/msg/Marker";
};
}
#endif
#ifndef DIMOS_MESSAGE_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95_TYPE
#define DIMOS_MESSAGE_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95_TYPE
namespace visualization_msgs::msg {
struct InteractiveMarkerControl {
static constexpr uint8_t INHERIT = 0;
static constexpr uint8_t FIXED = 1;
static constexpr uint8_t VIEW_FACING = 2;
static constexpr uint8_t NONE = 0;
static constexpr uint8_t MENU = 1;
static constexpr uint8_t BUTTON = 2;
static constexpr uint8_t MOVE_AXIS = 3;
static constexpr uint8_t MOVE_PLANE = 4;
static constexpr uint8_t ROTATE_AXIS = 5;
static constexpr uint8_t MOVE_ROTATE = 6;
static constexpr uint8_t MOVE_3D = 7;
static constexpr uint8_t ROTATE_3D = 8;
static constexpr uint8_t MOVE_ROTATE_3D = 9;
std::string name{};
geometry_msgs::msg::Quaternion orientation{};
uint8_t orientation_mode{};
uint8_t interaction_mode{};
bool always_visible{};
std::vector<visualization_msgs::msg::Marker> markers{};
bool independent_marker_orientation{};
std::string description{};
bool operator==(const InteractiveMarkerControl& other) const { return this->name == other.name && this->orientation == other.orientation && this->orientation_mode == other.orientation_mode && this->interaction_mode == other.interaction_mode && this->always_visible == other.always_visible && this->markers == other.markers && this->independent_marker_orientation == other.independent_marker_orientation && this->description == other.description; }
bool operator!=(const InteractiveMarkerControl& other) const { return !(*this == other); }
void validate() const {
orientation.validate();
for (const auto& item : markers) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerControl";
};
}
#endif
#ifndef DIMOS_MESSAGE_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618_TYPE
#define DIMOS_MESSAGE_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618_TYPE
namespace visualization_msgs::msg {
struct MenuEntry {
static constexpr uint8_t FEEDBACK = 0;
static constexpr uint8_t ROSRUN = 1;
static constexpr uint8_t ROSLAUNCH = 2;
uint32_t id{};
uint32_t parent_id{};
std::string title{};
std::string command{};
uint8_t command_type{};
bool operator==(const MenuEntry& other) const { return this->id == other.id && this->parent_id == other.parent_id && this->title == other.title && this->command == other.command && this->command_type == other.command_type; }
bool operator!=(const MenuEntry& other) const { return !(*this == other); }
void validate() const {
}
static constexpr const char* msg_name = "visualization_msgs/msg/MenuEntry";
};
}
#endif
#ifndef DIMOS_MESSAGE_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847_TYPE
#define DIMOS_MESSAGE_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847_TYPE
namespace visualization_msgs::msg {
struct InteractiveMarker {
std_msgs::msg::Header header{};
geometry_msgs::msg::Pose pose{};
std::string name{};
std::string description{};
float scale{};
std::vector<visualization_msgs::msg::MenuEntry> menu_entries{};
std::vector<visualization_msgs::msg::InteractiveMarkerControl> controls{};
bool operator==(const InteractiveMarker& other) const { return this->header == other.header && this->pose == other.pose && this->name == other.name && this->description == other.description && this->scale == other.scale && this->menu_entries == other.menu_entries && this->controls == other.controls; }
bool operator!=(const InteractiveMarker& other) const { return !(*this == other); }
void validate() const {
header.validate();
pose.validate();
for (const auto& item : menu_entries) { item.validate(); }
for (const auto& item : controls) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarker";
};
}
#endif
#ifndef DIMOS_MESSAGE_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E_TYPE
#define DIMOS_MESSAGE_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E_TYPE
namespace visualization_msgs::msg {
struct InteractiveMarkerFeedback {
static constexpr uint8_t KEEP_ALIVE = 0;
static constexpr uint8_t POSE_UPDATE = 1;
static constexpr uint8_t MENU_SELECT = 2;
static constexpr uint8_t BUTTON_CLICK = 3;
static constexpr uint8_t MOUSE_DOWN = 4;
static constexpr uint8_t MOUSE_UP = 5;
std_msgs::msg::Header header{};
std::string client_id{};
std::string marker_name{};
std::string control_name{};
uint8_t event_type{};
geometry_msgs::msg::Pose pose{};
uint32_t menu_entry_id{};
geometry_msgs::msg::Point mouse_point{};
bool mouse_point_valid{};
bool operator==(const InteractiveMarkerFeedback& other) const { return this->header == other.header && this->client_id == other.client_id && this->marker_name == other.marker_name && this->control_name == other.control_name && this->event_type == other.event_type && this->pose == other.pose && this->menu_entry_id == other.menu_entry_id && this->mouse_point == other.mouse_point && this->mouse_point_valid == other.mouse_point_valid; }
bool operator!=(const InteractiveMarkerFeedback& other) const { return !(*this == other); }
void validate() const {
header.validate();
pose.validate();
mouse_point.validate();
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerFeedback";
};
}
#endif
#ifndef DIMOS_MESSAGE_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4_TYPE
#define DIMOS_MESSAGE_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4_TYPE
namespace visualization_msgs::msg {
struct InteractiveMarkerInit {
std::string server_id{};
uint64_t seq_num{};
std::vector<visualization_msgs::msg::InteractiveMarker> markers{};
bool operator==(const InteractiveMarkerInit& other) const { return this->server_id == other.server_id && this->seq_num == other.seq_num && this->markers == other.markers; }
bool operator!=(const InteractiveMarkerInit& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : markers) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerInit";
};
}
#endif
#ifndef DIMOS_MESSAGE_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5_TYPE
#define DIMOS_MESSAGE_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5_TYPE
namespace visualization_msgs::msg {
struct InteractiveMarkerPose {
std_msgs::msg::Header header{};
geometry_msgs::msg::Pose pose{};
std::string name{};
bool operator==(const InteractiveMarkerPose& other) const { return this->header == other.header && this->pose == other.pose && this->name == other.name; }
bool operator!=(const InteractiveMarkerPose& other) const { return !(*this == other); }
void validate() const {
header.validate();
pose.validate();
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerPose";
};
}
#endif
#ifndef DIMOS_MESSAGE_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5_TYPE
#define DIMOS_MESSAGE_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5_TYPE
namespace visualization_msgs::msg {
struct InteractiveMarkerUpdate {
static constexpr uint8_t KEEP_ALIVE = 0;
static constexpr uint8_t UPDATE = 1;
std::string server_id{};
uint64_t seq_num{};
uint8_t type{};
std::vector<visualization_msgs::msg::InteractiveMarker> markers{};
std::vector<visualization_msgs::msg::InteractiveMarkerPose> poses{};
std::vector<std::string> erases{};
bool operator==(const InteractiveMarkerUpdate& other) const { return this->server_id == other.server_id && this->seq_num == other.seq_num && this->type == other.type && this->markers == other.markers && this->poses == other.poses && this->erases == other.erases; }
bool operator!=(const InteractiveMarkerUpdate& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : markers) { item.validate(); }
for (const auto& item : poses) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerUpdate";
};
}
#endif
#ifndef DIMOS_MESSAGE_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22_TYPE
#define DIMOS_MESSAGE_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22_TYPE
namespace visualization_msgs::msg {
struct MarkerArray {
std::vector<visualization_msgs::msg::Marker> markers{};
bool operator==(const MarkerArray& other) const { return this->markers == other.markers; }
bool operator!=(const MarkerArray& other) const { return !(*this == other); }
void validate() const {
for (const auto& item : markers) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/MarkerArray";
};
}
#endif
namespace eprosima::fastcdr {
#ifndef DIMOS_MESSAGE_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C_CODEC
#define DIMOS_MESSAGE_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const builtin_interfaces::msg::Duration& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.sec, alignment);
size += calculator.calculate_serialized_size(value.nanosec, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const builtin_interfaces::msg::Duration& value) {
cdr << value.sec;
cdr << value.nanosec;
}
template<> inline void deserialize(Cdr& cdr, builtin_interfaces::msg::Duration& value) {
cdr >> value.sec;
cdr >> value.nanosec;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4_CODEC
#define DIMOS_MESSAGE_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const builtin_interfaces::msg::Time& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.sec, alignment);
size += calculator.calculate_serialized_size(value.nanosec, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const builtin_interfaces::msg::Time& value) {
cdr << value.sec;
cdr << value.nanosec;
}
template<> inline void deserialize(Cdr& cdr, builtin_interfaces::msg::Time& value) {
cdr >> value.sec;
cdr >> value.nanosec;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D_CODEC
#define DIMOS_MESSAGE_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Header& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.stamp, alignment);
size += calculator.calculate_serialized_size(value.frame_id, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Header& value) {
cdr << value.stamp;
cdr << value.frame_id;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Header& value) {
cdr >> value.stamp;
cdr >> value.frame_id;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51_CODEC
#define DIMOS_MESSAGE_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Point2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Point2D& value) {
cdr << value.x;
cdr << value.y;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Point2D& value) {
cdr >> value.x;
cdr >> value.y;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97_CODEC
#define DIMOS_MESSAGE_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Pose2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.position, alignment);
size += calculator.calculate_serialized_size(value.theta, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Pose2D& value) {
cdr << value.position;
cdr << value.theta;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Pose2D& value) {
cdr >> value.position;
cdr >> value.theta;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA_CODEC
#define DIMOS_MESSAGE_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::BoundingBox2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.center, alignment);
size += calculator.calculate_serialized_size(value.size_x, alignment);
size += calculator.calculate_serialized_size(value.size_y, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::BoundingBox2D& value) {
cdr << value.center;
cdr << value.size_x;
cdr << value.size_y;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::BoundingBox2D& value) {
cdr >> value.center;
cdr >> value.size_x;
cdr >> value.size_y;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A_CODEC
#define DIMOS_MESSAGE_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::BoundingBox2DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.boxes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::BoundingBox2DArray& value) {
cdr << value.header;
cdr << value.boxes;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::BoundingBox2DArray& value) {
cdr >> value.header;
cdr >> value.boxes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C_CODEC
#define DIMOS_MESSAGE_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Point& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.z, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Point& value) {
cdr << value.x;
cdr << value.y;
cdr << value.z;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Point& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.z;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD_CODEC
#define DIMOS_MESSAGE_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Quaternion& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.z, alignment);
size += calculator.calculate_serialized_size(value.w, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Quaternion& value) {
cdr << value.x;
cdr << value.y;
cdr << value.z;
cdr << value.w;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Quaternion& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.z;
cdr >> value.w;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827_CODEC
#define DIMOS_MESSAGE_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Pose& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.position, alignment);
size += calculator.calculate_serialized_size(value.orientation, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Pose& value) {
cdr << value.position;
cdr << value.orientation;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Pose& value) {
cdr >> value.position;
cdr >> value.orientation;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02_CODEC
#define DIMOS_MESSAGE_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Vector3& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.z, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Vector3& value) {
cdr << value.x;
cdr << value.y;
cdr << value.z;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Vector3& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.z;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0_CODEC
#define DIMOS_MESSAGE_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::BoundingBox3D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.center, alignment);
size += calculator.calculate_serialized_size(value.size, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::BoundingBox3D& value) {
cdr << value.center;
cdr << value.size;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::BoundingBox3D& value) {
cdr >> value.center;
cdr >> value.size;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A_CODEC
#define DIMOS_MESSAGE_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::BoundingBox3DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.boxes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::BoundingBox3DArray& value) {
cdr << value.header;
cdr << value.boxes;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::BoundingBox3DArray& value) {
cdr >> value.header;
cdr >> value.boxes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87_CODEC
#define DIMOS_MESSAGE_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::EntityMarker& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.entity_id, alignment);
size += calculator.calculate_serialized_size(value.label, alignment);
size += calculator.calculate_serialized_size(value.entity_type, alignment);
size += calculator.calculate_serialized_size(value.position, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::EntityMarker& value) {
cdr << value.entity_id;
cdr << value.label;
cdr << value.entity_type;
cdr << value.position;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::EntityMarker& value) {
cdr >> value.entity_id;
cdr >> value.label;
cdr >> value.entity_type;
cdr >> value.position;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254_CODEC
#define DIMOS_MESSAGE_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::EntityMarkers& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.markers, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::EntityMarkers& value) {
cdr << value.header;
cdr << value.markers;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::EntityMarkers& value) {
cdr >> value.header;
cdr >> value.markers;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_78F4ED1DEB1B34DFDA6F24F5EE35F865A25466476BAECF67ABEE6DEBC2A7E65B_CODEC
#define DIMOS_MESSAGE_78F4ED1DEB1B34DFDA6F24F5EE35F865A25466476BAECF67ABEE6DEBC2A7E65B_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::EpisodeStatus& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.ts, alignment);
size += calculator.calculate_serialized_size(value.state, alignment);
size += calculator.calculate_serialized_size(value.episodes_saved, alignment);
size += calculator.calculate_serialized_size(value.episodes_discarded, alignment);
size += calculator.calculate_serialized_size(value.last_event, alignment);
size += calculator.calculate_serialized_size(value.task_label, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::EpisodeStatus& value) {
cdr << value.ts;
cdr << value.state;
cdr << value.episodes_saved;
cdr << value.episodes_discarded;
cdr << value.last_event;
cdr << value.task_label;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::EpisodeStatus& value) {
cdr >> value.ts;
cdr >> value.state;
cdr >> value.episodes_saved;
cdr >> value.episodes_discarded;
cdr >> value.last_event;
cdr >> value.task_label;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6_CODEC
#define DIMOS_MESSAGE_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::GraspCandidate& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.score, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::GraspCandidate& value) {
cdr << value.pose;
cdr << value.score;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::GraspCandidate& value) {
cdr >> value.pose;
cdr >> value.score;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC_CODEC
#define DIMOS_MESSAGE_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::GraspCandidateArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.candidates, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::GraspCandidateArray& value) {
cdr << value.header;
cdr << value.candidates;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::GraspCandidateArray& value) {
cdr >> value.header;
cdr >> value.candidates;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533_CODEC
#define DIMOS_MESSAGE_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::ImuInfo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.gyro_noise_density, alignment);
size += calculator.calculate_serialized_size(value.gyro_random_walk, alignment);
size += calculator.calculate_serialized_size(value.accel_noise_density, alignment);
size += calculator.calculate_serialized_size(value.accel_random_walk, alignment);
size += calculator.calculate_serialized_size(value.frequency, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::ImuInfo& value) {
cdr << value.header;
cdr << value.gyro_noise_density;
cdr << value.gyro_random_walk;
cdr << value.accel_noise_density;
cdr << value.accel_random_walk;
cdr << value.frequency;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::ImuInfo& value) {
cdr >> value.header;
cdr >> value.gyro_noise_density;
cdr >> value.gyro_random_walk;
cdr >> value.accel_noise_density;
cdr >> value.accel_random_walk;
cdr >> value.frequency;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02_CODEC
#define DIMOS_MESSAGE_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::JointCommand& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.positions, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::JointCommand& value) {
cdr << value.header;
cdr << value.positions;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::JointCommand& value) {
cdr >> value.header;
cdr >> value.positions;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2_CODEC
#define DIMOS_MESSAGE_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::LineSegment3D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.start, alignment);
size += calculator.calculate_serialized_size(value.end, alignment);
size += calculator.calculate_serialized_size(value.weight, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::LineSegment3D& value) {
cdr << value.start;
cdr << value.end;
cdr << value.weight;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::LineSegment3D& value) {
cdr >> value.start;
cdr >> value.end;
cdr >> value.weight;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C_CODEC
#define DIMOS_MESSAGE_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::LineSegments3D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.segments, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::LineSegments3D& value) {
cdr << value.header;
cdr << value.segments;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::LineSegments3D& value) {
cdr >> value.header;
cdr >> value.segments;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03_CODEC
#define DIMOS_MESSAGE_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::MotorCommandArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.q, alignment);
size += calculator.calculate_serialized_size(value.dq, alignment);
size += calculator.calculate_serialized_size(value.kp, alignment);
size += calculator.calculate_serialized_size(value.kd, alignment);
size += calculator.calculate_serialized_size(value.tau, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::MotorCommandArray& value) {
cdr << value.header;
cdr << value.q;
cdr << value.dq;
cdr << value.kp;
cdr << value.kd;
cdr << value.tau;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::MotorCommandArray& value) {
cdr >> value.header;
cdr >> value.q;
cdr >> value.dq;
cdr >> value.kp;
cdr >> value.kd;
cdr >> value.tau;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3_CODEC
#define DIMOS_MESSAGE_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::RobotState& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.state, alignment);
size += calculator.calculate_serialized_size(value.mode, alignment);
size += calculator.calculate_serialized_size(value.error_code, alignment);
size += calculator.calculate_serialized_size(value.warn_code, alignment);
size += calculator.calculate_serialized_size(value.cmdnum, alignment);
size += calculator.calculate_serialized_size(value.mt_brake, alignment);
size += calculator.calculate_serialized_size(value.mt_able, alignment);
size += calculator.calculate_serialized_size(value.tcp_pose, alignment);
size += calculator.calculate_serialized_size(value.tcp_offset, alignment);
size += calculator.calculate_serialized_size(value.joints, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::RobotState& value) {
cdr << value.header;
cdr << value.state;
cdr << value.mode;
cdr << value.error_code;
cdr << value.warn_code;
cdr << value.cmdnum;
cdr << value.mt_brake;
cdr << value.mt_able;
cdr << value.tcp_pose;
cdr << value.tcp_offset;
cdr << value.joints;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::RobotState& value) {
cdr >> value.header;
cdr >> value.state;
cdr >> value.mode;
cdr >> value.error_code;
cdr >> value.warn_code;
cdr >> value.cmdnum;
cdr >> value.mt_brake;
cdr >> value.mt_able;
cdr >> value.tcp_pose;
cdr >> value.tcp_offset;
cdr >> value.joints;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64_CODEC
#define DIMOS_MESSAGE_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::TrajectoryStatus& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.state, alignment);
size += calculator.calculate_serialized_size(value.progress, alignment);
size += calculator.calculate_serialized_size(value.time_elapsed, alignment);
size += calculator.calculate_serialized_size(value.time_remaining, alignment);
size += calculator.calculate_serialized_size(value.error, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::TrajectoryStatus& value) {
cdr << value.header;
cdr << value.state;
cdr << value.progress;
cdr << value.time_elapsed;
cdr << value.time_remaining;
cdr << value.error;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::TrajectoryStatus& value) {
cdr >> value.header;
cdr >> value.state;
cdr >> value.progress;
cdr >> value.time_elapsed;
cdr >> value.time_remaining;
cdr >> value.error;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125_CODEC
#define DIMOS_MESSAGE_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::VideoStats& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.fps, alignment);
size += calculator.calculate_serialized_size(value.kbps, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.loss_pct, alignment);
size += calculator.calculate_serialized_size(value.jitter_buffer_ms, alignment);
size += calculator.calculate_serialized_size(value.decode_ms, alignment);
size += calculator.calculate_serialized_size(value.frames_dropped, alignment);
size += calculator.calculate_serialized_size(value.freezes, alignment);
size += calculator.calculate_serialized_size(value.e2e_latency_ms, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::VideoStats& value) {
cdr << value.header;
cdr << value.fps;
cdr << value.kbps;
cdr << value.width;
cdr << value.height;
cdr << value.loss_pct;
cdr << value.jitter_buffer_ms;
cdr << value.decode_ms;
cdr << value.frames_dropped;
cdr << value.freezes;
cdr << value.e2e_latency_ms;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::VideoStats& value) {
cdr >> value.header;
cdr >> value.fps;
cdr >> value.kbps;
cdr >> value.width;
cdr >> value.height;
cdr >> value.loss_pct;
cdr >> value.jitter_buffer_ms;
cdr >> value.decode_ms;
cdr >> value.frames_dropped;
cdr >> value.freezes;
cdr >> value.e2e_latency_ms;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82_CODEC
#define DIMOS_MESSAGE_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const foxglove_msgs::msg::CompressedVideo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.timestamp, alignment);
size += calculator.calculate_serialized_size(value.frame_id, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
size += calculator.calculate_serialized_size(value.format, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const foxglove_msgs::msg::CompressedVideo& value) {
cdr << value.timestamp;
cdr << value.frame_id;
cdr << value.data;
cdr << value.format;
}
template<> inline void deserialize(Cdr& cdr, foxglove_msgs::msg::CompressedVideo& value) {
cdr >> value.timestamp;
cdr >> value.frame_id;
cdr >> value.data;
cdr >> value.format;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962_CODEC
#define DIMOS_MESSAGE_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Accel& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.linear, alignment);
size += calculator.calculate_serialized_size(value.angular, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Accel& value) {
cdr << value.linear;
cdr << value.angular;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Accel& value) {
cdr >> value.linear;
cdr >> value.angular;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3_CODEC
#define DIMOS_MESSAGE_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::AccelStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.accel, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::AccelStamped& value) {
cdr << value.header;
cdr << value.accel;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::AccelStamped& value) {
cdr >> value.header;
cdr >> value.accel;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7_CODEC
#define DIMOS_MESSAGE_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::AccelWithCovariance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.accel, alignment);
size += calculator.calculate_serialized_size(value.covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::AccelWithCovariance& value) {
cdr << value.accel;
cdr << value.covariance;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::AccelWithCovariance& value) {
cdr >> value.accel;
cdr >> value.covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283_CODEC
#define DIMOS_MESSAGE_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::AccelWithCovarianceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.accel, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::AccelWithCovarianceStamped& value) {
cdr << value.header;
cdr << value.accel;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::AccelWithCovarianceStamped& value) {
cdr >> value.header;
cdr >> value.accel;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE_CODEC
#define DIMOS_MESSAGE_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Inertia& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.m, alignment);
size += calculator.calculate_serialized_size(value.com, alignment);
size += calculator.calculate_serialized_size(value.ixx, alignment);
size += calculator.calculate_serialized_size(value.ixy, alignment);
size += calculator.calculate_serialized_size(value.ixz, alignment);
size += calculator.calculate_serialized_size(value.iyy, alignment);
size += calculator.calculate_serialized_size(value.iyz, alignment);
size += calculator.calculate_serialized_size(value.izz, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Inertia& value) {
cdr << value.m;
cdr << value.com;
cdr << value.ixx;
cdr << value.ixy;
cdr << value.ixz;
cdr << value.iyy;
cdr << value.iyz;
cdr << value.izz;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Inertia& value) {
cdr >> value.m;
cdr >> value.com;
cdr >> value.ixx;
cdr >> value.ixy;
cdr >> value.ixz;
cdr >> value.iyy;
cdr >> value.iyz;
cdr >> value.izz;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF_CODEC
#define DIMOS_MESSAGE_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::InertiaStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.inertia, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::InertiaStamped& value) {
cdr << value.header;
cdr << value.inertia;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::InertiaStamped& value) {
cdr >> value.header;
cdr >> value.inertia;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6_CODEC
#define DIMOS_MESSAGE_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Point32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.z, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Point32& value) {
cdr << value.x;
cdr << value.y;
cdr << value.z;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Point32& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.z;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328_CODEC
#define DIMOS_MESSAGE_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PointStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.point, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PointStamped& value) {
cdr << value.header;
cdr << value.point;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PointStamped& value) {
cdr >> value.header;
cdr >> value.point;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469_CODEC
#define DIMOS_MESSAGE_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Polygon& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.points, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Polygon& value) {
cdr << value.points;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Polygon& value) {
cdr >> value.points;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C_CODEC
#define DIMOS_MESSAGE_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PolygonInstance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.polygon, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PolygonInstance& value) {
cdr << value.polygon;
cdr << value.id;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PolygonInstance& value) {
cdr >> value.polygon;
cdr >> value.id;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22_CODEC
#define DIMOS_MESSAGE_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PolygonInstanceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.polygon, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PolygonInstanceStamped& value) {
cdr << value.header;
cdr << value.polygon;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PolygonInstanceStamped& value) {
cdr >> value.header;
cdr >> value.polygon;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F_CODEC
#define DIMOS_MESSAGE_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PolygonStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.polygon, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PolygonStamped& value) {
cdr << value.header;
cdr << value.polygon;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PolygonStamped& value) {
cdr >> value.header;
cdr >> value.polygon;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8_CODEC
#define DIMOS_MESSAGE_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Pose2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.theta, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Pose2D& value) {
cdr << value.x;
cdr << value.y;
cdr << value.theta;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Pose2D& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.theta;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3_CODEC
#define DIMOS_MESSAGE_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PoseArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.poses, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PoseArray& value) {
cdr << value.header;
cdr << value.poses;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PoseArray& value) {
cdr >> value.header;
cdr >> value.poses;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F_CODEC
#define DIMOS_MESSAGE_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PoseStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PoseStamped& value) {
cdr << value.header;
cdr << value.pose;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PoseStamped& value) {
cdr >> value.header;
cdr >> value.pose;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A_CODEC
#define DIMOS_MESSAGE_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PoseWithCovariance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PoseWithCovariance& value) {
cdr << value.pose;
cdr << value.covariance;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PoseWithCovariance& value) {
cdr >> value.pose;
cdr >> value.covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B_CODEC
#define DIMOS_MESSAGE_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PoseWithCovarianceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PoseWithCovarianceStamped& value) {
cdr << value.header;
cdr << value.pose;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PoseWithCovarianceStamped& value) {
cdr >> value.header;
cdr >> value.pose;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5_CODEC
#define DIMOS_MESSAGE_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::QuaternionStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.quaternion, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::QuaternionStamped& value) {
cdr << value.header;
cdr << value.quaternion;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::QuaternionStamped& value) {
cdr >> value.header;
cdr >> value.quaternion;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238_CODEC
#define DIMOS_MESSAGE_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Transform& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.translation, alignment);
size += calculator.calculate_serialized_size(value.rotation, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Transform& value) {
cdr << value.translation;
cdr << value.rotation;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Transform& value) {
cdr >> value.translation;
cdr >> value.rotation;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA_CODEC
#define DIMOS_MESSAGE_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::TransformStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.child_frame_id, alignment);
size += calculator.calculate_serialized_size(value.transform, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::TransformStamped& value) {
cdr << value.header;
cdr << value.child_frame_id;
cdr << value.transform;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::TransformStamped& value) {
cdr >> value.header;
cdr >> value.child_frame_id;
cdr >> value.transform;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85_CODEC
#define DIMOS_MESSAGE_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Twist& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.linear, alignment);
size += calculator.calculate_serialized_size(value.angular, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Twist& value) {
cdr << value.linear;
cdr << value.angular;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Twist& value) {
cdr >> value.linear;
cdr >> value.angular;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C_CODEC
#define DIMOS_MESSAGE_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::TwistStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.twist, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::TwistStamped& value) {
cdr << value.header;
cdr << value.twist;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::TwistStamped& value) {
cdr >> value.header;
cdr >> value.twist;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD_CODEC
#define DIMOS_MESSAGE_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::TwistWithCovariance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.twist, alignment);
size += calculator.calculate_serialized_size(value.covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::TwistWithCovariance& value) {
cdr << value.twist;
cdr << value.covariance;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::TwistWithCovariance& value) {
cdr >> value.twist;
cdr >> value.covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6_CODEC
#define DIMOS_MESSAGE_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::TwistWithCovarianceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.twist, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::TwistWithCovarianceStamped& value) {
cdr << value.header;
cdr << value.twist;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::TwistWithCovarianceStamped& value) {
cdr >> value.header;
cdr >> value.twist;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75_CODEC
#define DIMOS_MESSAGE_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Vector3Stamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.vector, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Vector3Stamped& value) {
cdr << value.header;
cdr << value.vector;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Vector3Stamped& value) {
cdr >> value.header;
cdr >> value.vector;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387_CODEC
#define DIMOS_MESSAGE_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::VelocityStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.body_frame_id, alignment);
size += calculator.calculate_serialized_size(value.reference_frame_id, alignment);
size += calculator.calculate_serialized_size(value.velocity, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::VelocityStamped& value) {
cdr << value.header;
cdr << value.body_frame_id;
cdr << value.reference_frame_id;
cdr << value.velocity;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::VelocityStamped& value) {
cdr >> value.header;
cdr >> value.body_frame_id;
cdr >> value.reference_frame_id;
cdr >> value.velocity;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732_CODEC
#define DIMOS_MESSAGE_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::VelocityWithCovarianceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.body_frame_id, alignment);
size += calculator.calculate_serialized_size(value.reference_frame_id, alignment);
size += calculator.calculate_serialized_size(value.velocity, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::VelocityWithCovarianceStamped& value) {
cdr << value.header;
cdr << value.body_frame_id;
cdr << value.reference_frame_id;
cdr << value.velocity;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::VelocityWithCovarianceStamped& value) {
cdr >> value.header;
cdr >> value.body_frame_id;
cdr >> value.reference_frame_id;
cdr >> value.velocity;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12_CODEC
#define DIMOS_MESSAGE_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Wrench& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.force, alignment);
size += calculator.calculate_serialized_size(value.torque, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Wrench& value) {
cdr << value.force;
cdr << value.torque;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Wrench& value) {
cdr >> value.force;
cdr >> value.torque;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB_CODEC
#define DIMOS_MESSAGE_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::WrenchStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.wrench, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::WrenchStamped& value) {
cdr << value.header;
cdr << value.wrench;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::WrenchStamped& value) {
cdr >> value.header;
cdr >> value.wrench;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0_CODEC
#define DIMOS_MESSAGE_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::Goals& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.goals, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::Goals& value) {
cdr << value.header;
cdr << value.goals;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::Goals& value) {
cdr >> value.header;
cdr >> value.goals;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B_CODEC
#define DIMOS_MESSAGE_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::GridCells& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.cell_width, alignment);
size += calculator.calculate_serialized_size(value.cell_height, alignment);
size += calculator.calculate_serialized_size(value.cells, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::GridCells& value) {
cdr << value.header;
cdr << value.cell_width;
cdr << value.cell_height;
cdr << value.cells;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::GridCells& value) {
cdr >> value.header;
cdr >> value.cell_width;
cdr >> value.cell_height;
cdr >> value.cells;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0_CODEC
#define DIMOS_MESSAGE_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::MapMetaData& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.map_load_time, alignment);
size += calculator.calculate_serialized_size(value.resolution, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.origin, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::MapMetaData& value) {
cdr << value.map_load_time;
cdr << value.resolution;
cdr << value.width;
cdr << value.height;
cdr << value.origin;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::MapMetaData& value) {
cdr >> value.map_load_time;
cdr >> value.resolution;
cdr >> value.width;
cdr >> value.height;
cdr >> value.origin;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D_CODEC
#define DIMOS_MESSAGE_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::OccupancyGrid& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.info, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::OccupancyGrid& value) {
cdr << value.header;
cdr << value.info;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::OccupancyGrid& value) {
cdr >> value.header;
cdr >> value.info;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24_CODEC
#define DIMOS_MESSAGE_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::Odometry& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.child_frame_id, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.twist, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::Odometry& value) {
cdr << value.header;
cdr << value.child_frame_id;
cdr << value.pose;
cdr << value.twist;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::Odometry& value) {
cdr >> value.header;
cdr >> value.child_frame_id;
cdr >> value.pose;
cdr >> value.twist;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7_CODEC
#define DIMOS_MESSAGE_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::Path& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.poses, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::Path& value) {
cdr << value.header;
cdr << value.poses;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::Path& value) {
cdr >> value.header;
cdr >> value.poses;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96_CODEC
#define DIMOS_MESSAGE_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::TrajectoryPoint& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.velocity, alignment);
size += calculator.calculate_serialized_size(value.acceleration, alignment);
size += calculator.calculate_serialized_size(value.effort, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::TrajectoryPoint& value) {
cdr << value.header;
cdr << value.pose;
cdr << value.velocity;
cdr << value.acceleration;
cdr << value.effort;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::TrajectoryPoint& value) {
cdr >> value.header;
cdr >> value.pose;
cdr >> value.velocity;
cdr >> value.acceleration;
cdr >> value.effort;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA_CODEC
#define DIMOS_MESSAGE_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::Trajectory& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::Trajectory& value) {
cdr << value.header;
cdr << value.points;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::Trajectory& value) {
cdr >> value.header;
cdr >> value.points;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5_CODEC
#define DIMOS_MESSAGE_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::BatteryState& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.voltage, alignment);
size += calculator.calculate_serialized_size(value.temperature, alignment);
size += calculator.calculate_serialized_size(value.current, alignment);
size += calculator.calculate_serialized_size(value.charge, alignment);
size += calculator.calculate_serialized_size(value.capacity, alignment);
size += calculator.calculate_serialized_size(value.design_capacity, alignment);
size += calculator.calculate_serialized_size(value.percentage, alignment);
size += calculator.calculate_serialized_size(value.power_supply_status, alignment);
size += calculator.calculate_serialized_size(value.power_supply_health, alignment);
size += calculator.calculate_serialized_size(value.power_supply_technology, alignment);
size += calculator.calculate_serialized_size(value.present, alignment);
size += calculator.calculate_serialized_size(value.cell_voltage, alignment);
size += calculator.calculate_serialized_size(value.cell_temperature, alignment);
size += calculator.calculate_serialized_size(value.location, alignment);
size += calculator.calculate_serialized_size(value.serial_number, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::BatteryState& value) {
cdr << value.header;
cdr << value.voltage;
cdr << value.temperature;
cdr << value.current;
cdr << value.charge;
cdr << value.capacity;
cdr << value.design_capacity;
cdr << value.percentage;
cdr << value.power_supply_status;
cdr << value.power_supply_health;
cdr << value.power_supply_technology;
cdr << value.present;
cdr << value.cell_voltage;
cdr << value.cell_temperature;
cdr << value.location;
cdr << value.serial_number;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::BatteryState& value) {
cdr >> value.header;
cdr >> value.voltage;
cdr >> value.temperature;
cdr >> value.current;
cdr >> value.charge;
cdr >> value.capacity;
cdr >> value.design_capacity;
cdr >> value.percentage;
cdr >> value.power_supply_status;
cdr >> value.power_supply_health;
cdr >> value.power_supply_technology;
cdr >> value.present;
cdr >> value.cell_voltage;
cdr >> value.cell_temperature;
cdr >> value.location;
cdr >> value.serial_number;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62_CODEC
#define DIMOS_MESSAGE_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::RegionOfInterest& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x_offset, alignment);
size += calculator.calculate_serialized_size(value.y_offset, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.do_rectify, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::RegionOfInterest& value) {
cdr << value.x_offset;
cdr << value.y_offset;
cdr << value.height;
cdr << value.width;
cdr << value.do_rectify;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::RegionOfInterest& value) {
cdr >> value.x_offset;
cdr >> value.y_offset;
cdr >> value.height;
cdr >> value.width;
cdr >> value.do_rectify;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5_CODEC
#define DIMOS_MESSAGE_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::CameraInfo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.distortion_model, alignment);
size += calculator.calculate_serialized_size(value.d, alignment);
size += calculator.calculate_serialized_size(value.k, alignment);
size += calculator.calculate_serialized_size(value.r, alignment);
size += calculator.calculate_serialized_size(value.p, alignment);
size += calculator.calculate_serialized_size(value.binning_x, alignment);
size += calculator.calculate_serialized_size(value.binning_y, alignment);
size += calculator.calculate_serialized_size(value.roi, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::CameraInfo& value) {
cdr << value.header;
cdr << value.height;
cdr << value.width;
cdr << value.distortion_model;
cdr << value.d;
cdr << value.k;
cdr << value.r;
cdr << value.p;
cdr << value.binning_x;
cdr << value.binning_y;
cdr << value.roi;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::CameraInfo& value) {
cdr >> value.header;
cdr >> value.height;
cdr >> value.width;
cdr >> value.distortion_model;
cdr >> value.d;
cdr >> value.k;
cdr >> value.r;
cdr >> value.p;
cdr >> value.binning_x;
cdr >> value.binning_y;
cdr >> value.roi;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6_CODEC
#define DIMOS_MESSAGE_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::ChannelFloat32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.values, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::ChannelFloat32& value) {
cdr << value.name;
cdr << value.values;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::ChannelFloat32& value) {
cdr >> value.name;
cdr >> value.values;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A_CODEC
#define DIMOS_MESSAGE_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::CompressedImage& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.format, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::CompressedImage& value) {
cdr << value.header;
cdr << value.format;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::CompressedImage& value) {
cdr >> value.header;
cdr >> value.format;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87_CODEC
#define DIMOS_MESSAGE_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::FluidPressure& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.fluid_pressure, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::FluidPressure& value) {
cdr << value.header;
cdr << value.fluid_pressure;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::FluidPressure& value) {
cdr >> value.header;
cdr >> value.fluid_pressure;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733_CODEC
#define DIMOS_MESSAGE_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Illuminance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.illuminance, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Illuminance& value) {
cdr << value.header;
cdr << value.illuminance;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Illuminance& value) {
cdr >> value.header;
cdr >> value.illuminance;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25_CODEC
#define DIMOS_MESSAGE_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Image& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.encoding, alignment);
size += calculator.calculate_serialized_size(value.is_bigendian, alignment);
size += calculator.calculate_serialized_size(value.step, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Image& value) {
cdr << value.header;
cdr << value.height;
cdr << value.width;
cdr << value.encoding;
cdr << value.is_bigendian;
cdr << value.step;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Image& value) {
cdr >> value.header;
cdr >> value.height;
cdr >> value.width;
cdr >> value.encoding;
cdr >> value.is_bigendian;
cdr >> value.step;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75_CODEC
#define DIMOS_MESSAGE_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Imu& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.orientation, alignment);
size += calculator.calculate_serialized_size(value.orientation_covariance, alignment);
size += calculator.calculate_serialized_size(value.angular_velocity, alignment);
size += calculator.calculate_serialized_size(value.angular_velocity_covariance, alignment);
size += calculator.calculate_serialized_size(value.linear_acceleration, alignment);
size += calculator.calculate_serialized_size(value.linear_acceleration_covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Imu& value) {
cdr << value.header;
cdr << value.orientation;
cdr << value.orientation_covariance;
cdr << value.angular_velocity;
cdr << value.angular_velocity_covariance;
cdr << value.linear_acceleration;
cdr << value.linear_acceleration_covariance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Imu& value) {
cdr >> value.header;
cdr >> value.orientation;
cdr >> value.orientation_covariance;
cdr >> value.angular_velocity;
cdr >> value.angular_velocity_covariance;
cdr >> value.linear_acceleration;
cdr >> value.linear_acceleration_covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752_CODEC
#define DIMOS_MESSAGE_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::JointState& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.position, alignment);
size += calculator.calculate_serialized_size(value.velocity, alignment);
size += calculator.calculate_serialized_size(value.effort, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::JointState& value) {
cdr << value.header;
cdr << value.name;
cdr << value.position;
cdr << value.velocity;
cdr << value.effort;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::JointState& value) {
cdr >> value.header;
cdr >> value.name;
cdr >> value.position;
cdr >> value.velocity;
cdr >> value.effort;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8_CODEC
#define DIMOS_MESSAGE_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Joy& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.axes, alignment);
size += calculator.calculate_serialized_size(value.buttons, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Joy& value) {
cdr << value.header;
cdr << value.axes;
cdr << value.buttons;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Joy& value) {
cdr >> value.header;
cdr >> value.axes;
cdr >> value.buttons;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F_CODEC
#define DIMOS_MESSAGE_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::JoyFeedback& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
size += calculator.calculate_serialized_size(value.intensity, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::JoyFeedback& value) {
cdr << value.type;
cdr << value.id;
cdr << value.intensity;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::JoyFeedback& value) {
cdr >> value.type;
cdr >> value.id;
cdr >> value.intensity;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF_CODEC
#define DIMOS_MESSAGE_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::JoyFeedbackArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.array, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::JoyFeedbackArray& value) {
cdr << value.array;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::JoyFeedbackArray& value) {
cdr >> value.array;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71_CODEC
#define DIMOS_MESSAGE_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::LaserEcho& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.echoes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::LaserEcho& value) {
cdr << value.echoes;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::LaserEcho& value) {
cdr >> value.echoes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419_CODEC
#define DIMOS_MESSAGE_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::LaserScan& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.angle_min, alignment);
size += calculator.calculate_serialized_size(value.angle_max, alignment);
size += calculator.calculate_serialized_size(value.angle_increment, alignment);
size += calculator.calculate_serialized_size(value.time_increment, alignment);
size += calculator.calculate_serialized_size(value.scan_time, alignment);
size += calculator.calculate_serialized_size(value.range_min, alignment);
size += calculator.calculate_serialized_size(value.range_max, alignment);
size += calculator.calculate_serialized_size(value.ranges, alignment);
size += calculator.calculate_serialized_size(value.intensities, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::LaserScan& value) {
cdr << value.header;
cdr << value.angle_min;
cdr << value.angle_max;
cdr << value.angle_increment;
cdr << value.time_increment;
cdr << value.scan_time;
cdr << value.range_min;
cdr << value.range_max;
cdr << value.ranges;
cdr << value.intensities;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::LaserScan& value) {
cdr >> value.header;
cdr >> value.angle_min;
cdr >> value.angle_max;
cdr >> value.angle_increment;
cdr >> value.time_increment;
cdr >> value.scan_time;
cdr >> value.range_min;
cdr >> value.range_max;
cdr >> value.ranges;
cdr >> value.intensities;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E_CODEC
#define DIMOS_MESSAGE_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::MagneticField& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.magnetic_field, alignment);
size += calculator.calculate_serialized_size(value.magnetic_field_covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::MagneticField& value) {
cdr << value.header;
cdr << value.magnetic_field;
cdr << value.magnetic_field_covariance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::MagneticField& value) {
cdr >> value.header;
cdr >> value.magnetic_field;
cdr >> value.magnetic_field_covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C_CODEC
#define DIMOS_MESSAGE_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::MultiDOFJointState& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.joint_names, alignment);
size += calculator.calculate_serialized_size(value.transforms, alignment);
size += calculator.calculate_serialized_size(value.twist, alignment);
size += calculator.calculate_serialized_size(value.wrench, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::MultiDOFJointState& value) {
cdr << value.header;
cdr << value.joint_names;
cdr << value.transforms;
cdr << value.twist;
cdr << value.wrench;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::MultiDOFJointState& value) {
cdr >> value.header;
cdr >> value.joint_names;
cdr >> value.transforms;
cdr >> value.twist;
cdr >> value.wrench;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF_CODEC
#define DIMOS_MESSAGE_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::MultiEchoLaserScan& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.angle_min, alignment);
size += calculator.calculate_serialized_size(value.angle_max, alignment);
size += calculator.calculate_serialized_size(value.angle_increment, alignment);
size += calculator.calculate_serialized_size(value.time_increment, alignment);
size += calculator.calculate_serialized_size(value.scan_time, alignment);
size += calculator.calculate_serialized_size(value.range_min, alignment);
size += calculator.calculate_serialized_size(value.range_max, alignment);
size += calculator.calculate_serialized_size(value.ranges, alignment);
size += calculator.calculate_serialized_size(value.intensities, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::MultiEchoLaserScan& value) {
cdr << value.header;
cdr << value.angle_min;
cdr << value.angle_max;
cdr << value.angle_increment;
cdr << value.time_increment;
cdr << value.scan_time;
cdr << value.range_min;
cdr << value.range_max;
cdr << value.ranges;
cdr << value.intensities;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::MultiEchoLaserScan& value) {
cdr >> value.header;
cdr >> value.angle_min;
cdr >> value.angle_max;
cdr >> value.angle_increment;
cdr >> value.time_increment;
cdr >> value.scan_time;
cdr >> value.range_min;
cdr >> value.range_max;
cdr >> value.ranges;
cdr >> value.intensities;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2_CODEC
#define DIMOS_MESSAGE_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::NavSatStatus& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.status, alignment);
size += calculator.calculate_serialized_size(value.service, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::NavSatStatus& value) {
cdr << value.status;
cdr << value.service;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::NavSatStatus& value) {
cdr >> value.status;
cdr >> value.service;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9_CODEC
#define DIMOS_MESSAGE_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::NavSatFix& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.status, alignment);
size += calculator.calculate_serialized_size(value.latitude, alignment);
size += calculator.calculate_serialized_size(value.longitude, alignment);
size += calculator.calculate_serialized_size(value.altitude, alignment);
size += calculator.calculate_serialized_size(value.position_covariance, alignment);
size += calculator.calculate_serialized_size(value.position_covariance_type, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::NavSatFix& value) {
cdr << value.header;
cdr << value.status;
cdr << value.latitude;
cdr << value.longitude;
cdr << value.altitude;
cdr << value.position_covariance;
cdr << value.position_covariance_type;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::NavSatFix& value) {
cdr >> value.header;
cdr >> value.status;
cdr >> value.latitude;
cdr >> value.longitude;
cdr >> value.altitude;
cdr >> value.position_covariance;
cdr >> value.position_covariance_type;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D_CODEC
#define DIMOS_MESSAGE_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::PointCloud& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
size += calculator.calculate_serialized_size(value.channels, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::PointCloud& value) {
cdr << value.header;
cdr << value.points;
cdr << value.channels;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::PointCloud& value) {
cdr >> value.header;
cdr >> value.points;
cdr >> value.channels;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018_CODEC
#define DIMOS_MESSAGE_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::PointField& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.offset, alignment);
size += calculator.calculate_serialized_size(value.datatype, alignment);
size += calculator.calculate_serialized_size(value.count, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::PointField& value) {
cdr << value.name;
cdr << value.offset;
cdr << value.datatype;
cdr << value.count;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::PointField& value) {
cdr >> value.name;
cdr >> value.offset;
cdr >> value.datatype;
cdr >> value.count;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973_CODEC
#define DIMOS_MESSAGE_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::PointCloud2& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.fields, alignment);
size += calculator.calculate_serialized_size(value.is_bigendian, alignment);
size += calculator.calculate_serialized_size(value.point_step, alignment);
size += calculator.calculate_serialized_size(value.row_step, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
size += calculator.calculate_serialized_size(value.is_dense, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::PointCloud2& value) {
cdr << value.header;
cdr << value.height;
cdr << value.width;
cdr << value.fields;
cdr << value.is_bigendian;
cdr << value.point_step;
cdr << value.row_step;
cdr << value.data;
cdr << value.is_dense;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::PointCloud2& value) {
cdr >> value.header;
cdr >> value.height;
cdr >> value.width;
cdr >> value.fields;
cdr >> value.is_bigendian;
cdr >> value.point_step;
cdr >> value.row_step;
cdr >> value.data;
cdr >> value.is_dense;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579_CODEC
#define DIMOS_MESSAGE_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Range& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.radiation_type, alignment);
size += calculator.calculate_serialized_size(value.field_of_view, alignment);
size += calculator.calculate_serialized_size(value.min_range, alignment);
size += calculator.calculate_serialized_size(value.max_range, alignment);
size += calculator.calculate_serialized_size(value.range, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Range& value) {
cdr << value.header;
cdr << value.radiation_type;
cdr << value.field_of_view;
cdr << value.min_range;
cdr << value.max_range;
cdr << value.range;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Range& value) {
cdr >> value.header;
cdr >> value.radiation_type;
cdr >> value.field_of_view;
cdr >> value.min_range;
cdr >> value.max_range;
cdr >> value.range;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0_CODEC
#define DIMOS_MESSAGE_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::RelativeHumidity& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.relative_humidity, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::RelativeHumidity& value) {
cdr << value.header;
cdr << value.relative_humidity;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::RelativeHumidity& value) {
cdr >> value.header;
cdr >> value.relative_humidity;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D_CODEC
#define DIMOS_MESSAGE_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Temperature& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.temperature, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Temperature& value) {
cdr << value.header;
cdr << value.temperature;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Temperature& value) {
cdr >> value.header;
cdr >> value.temperature;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430_CODEC
#define DIMOS_MESSAGE_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::TimeReference& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.time_ref, alignment);
size += calculator.calculate_serialized_size(value.source, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::TimeReference& value) {
cdr << value.header;
cdr << value.time_ref;
cdr << value.source;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::TimeReference& value) {
cdr >> value.header;
cdr >> value.time_ref;
cdr >> value.source;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50_CODEC
#define DIMOS_MESSAGE_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const shape_msgs::msg::MeshTriangle& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.vertex_indices, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const shape_msgs::msg::MeshTriangle& value) {
cdr << value.vertex_indices;
}
template<> inline void deserialize(Cdr& cdr, shape_msgs::msg::MeshTriangle& value) {
cdr >> value.vertex_indices;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68_CODEC
#define DIMOS_MESSAGE_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const shape_msgs::msg::Mesh& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.triangles, alignment);
size += calculator.calculate_serialized_size(value.vertices, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const shape_msgs::msg::Mesh& value) {
cdr << value.triangles;
cdr << value.vertices;
}
template<> inline void deserialize(Cdr& cdr, shape_msgs::msg::Mesh& value) {
cdr >> value.triangles;
cdr >> value.vertices;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5_CODEC
#define DIMOS_MESSAGE_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const shape_msgs::msg::Plane& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.coef, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const shape_msgs::msg::Plane& value) {
cdr << value.coef;
}
template<> inline void deserialize(Cdr& cdr, shape_msgs::msg::Plane& value) {
cdr >> value.coef;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5_CODEC
#define DIMOS_MESSAGE_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const shape_msgs::msg::SolidPrimitive& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.dimensions, alignment);
size += calculator.calculate_serialized_size(value.polygon, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const shape_msgs::msg::SolidPrimitive& value) {
cdr << value.type;
cdr << value.dimensions;
cdr << value.polygon;
}
template<> inline void deserialize(Cdr& cdr, shape_msgs::msg::SolidPrimitive& value) {
cdr >> value.type;
cdr >> value.dimensions;
cdr >> value.polygon;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B_CODEC
#define DIMOS_MESSAGE_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Bool& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Bool& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Bool& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141_CODEC
#define DIMOS_MESSAGE_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Byte& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Byte& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Byte& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2_CODEC
#define DIMOS_MESSAGE_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::MultiArrayDimension& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.label, alignment);
size += calculator.calculate_serialized_size(value.size, alignment);
size += calculator.calculate_serialized_size(value.stride, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::MultiArrayDimension& value) {
cdr << value.label;
cdr << value.size;
cdr << value.stride;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::MultiArrayDimension& value) {
cdr >> value.label;
cdr >> value.size;
cdr >> value.stride;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8_CODEC
#define DIMOS_MESSAGE_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::MultiArrayLayout& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.dim, alignment);
size += calculator.calculate_serialized_size(value.data_offset, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::MultiArrayLayout& value) {
cdr << value.dim;
cdr << value.data_offset;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::MultiArrayLayout& value) {
cdr >> value.dim;
cdr >> value.data_offset;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39_CODEC
#define DIMOS_MESSAGE_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::ByteMultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::ByteMultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::ByteMultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7_CODEC
#define DIMOS_MESSAGE_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Char& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Char& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Char& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F_CODEC
#define DIMOS_MESSAGE_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::ColorRGBA& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.r, alignment);
size += calculator.calculate_serialized_size(value.g, alignment);
size += calculator.calculate_serialized_size(value.b, alignment);
size += calculator.calculate_serialized_size(value.a, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::ColorRGBA& value) {
cdr << value.r;
cdr << value.g;
cdr << value.b;
cdr << value.a;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::ColorRGBA& value) {
cdr >> value.r;
cdr >> value.g;
cdr >> value.b;
cdr >> value.a;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380_CODEC
#define DIMOS_MESSAGE_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Empty& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(uint8_t{0}, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Empty& value) {
cdr << uint8_t{0};
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Empty& value) {
uint8_t unused; cdr >> unused;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC_CODEC
#define DIMOS_MESSAGE_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Float32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Float32& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Float32& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1_CODEC
#define DIMOS_MESSAGE_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Float32MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Float32MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Float32MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33_CODEC
#define DIMOS_MESSAGE_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Float64& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Float64& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Float64& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330_CODEC
#define DIMOS_MESSAGE_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Float64MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Float64MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Float64MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731_CODEC
#define DIMOS_MESSAGE_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int16& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int16& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int16& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119_CODEC
#define DIMOS_MESSAGE_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int16MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int16MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int16MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488_CODEC
#define DIMOS_MESSAGE_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int32& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int32& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969_CODEC
#define DIMOS_MESSAGE_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int32MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int32MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int32MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D_CODEC
#define DIMOS_MESSAGE_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int64& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int64& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int64& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F_CODEC
#define DIMOS_MESSAGE_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int64MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int64MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int64MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575_CODEC
#define DIMOS_MESSAGE_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int8& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int8& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int8& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C_CODEC
#define DIMOS_MESSAGE_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int8MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int8MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int8MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1_CODEC
#define DIMOS_MESSAGE_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::String& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::String& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::String& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5_CODEC
#define DIMOS_MESSAGE_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt16& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt16& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt16& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81_CODEC
#define DIMOS_MESSAGE_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt16MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt16MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt16MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F_CODEC
#define DIMOS_MESSAGE_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt32& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt32& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD_CODEC
#define DIMOS_MESSAGE_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt32MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt32MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt32MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7_CODEC
#define DIMOS_MESSAGE_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt64& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt64& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt64& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95_CODEC
#define DIMOS_MESSAGE_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt64MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt64MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt64MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4_CODEC
#define DIMOS_MESSAGE_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt8& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt8& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt8& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636_CODEC
#define DIMOS_MESSAGE_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt8MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt8MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt8MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C_CODEC
#define DIMOS_MESSAGE_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const tf2_msgs::msg::TF2Error& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.error, alignment);
size += calculator.calculate_serialized_size(value.error_string, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const tf2_msgs::msg::TF2Error& value) {
cdr << value.error;
cdr << value.error_string;
}
template<> inline void deserialize(Cdr& cdr, tf2_msgs::msg::TF2Error& value) {
cdr >> value.error;
cdr >> value.error_string;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A_CODEC
#define DIMOS_MESSAGE_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const tf2_msgs::msg::TFMessage& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.transforms, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const tf2_msgs::msg::TFMessage& value) {
cdr << value.transforms;
}
template<> inline void deserialize(Cdr& cdr, tf2_msgs::msg::TFMessage& value) {
cdr >> value.transforms;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848_CODEC
#define DIMOS_MESSAGE_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const trajectory_msgs::msg::JointTrajectoryPoint& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.positions, alignment);
size += calculator.calculate_serialized_size(value.velocities, alignment);
size += calculator.calculate_serialized_size(value.accelerations, alignment);
size += calculator.calculate_serialized_size(value.effort, alignment);
size += calculator.calculate_serialized_size(value.time_from_start, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const trajectory_msgs::msg::JointTrajectoryPoint& value) {
cdr << value.positions;
cdr << value.velocities;
cdr << value.accelerations;
cdr << value.effort;
cdr << value.time_from_start;
}
template<> inline void deserialize(Cdr& cdr, trajectory_msgs::msg::JointTrajectoryPoint& value) {
cdr >> value.positions;
cdr >> value.velocities;
cdr >> value.accelerations;
cdr >> value.effort;
cdr >> value.time_from_start;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56_CODEC
#define DIMOS_MESSAGE_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const trajectory_msgs::msg::JointTrajectory& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.joint_names, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const trajectory_msgs::msg::JointTrajectory& value) {
cdr << value.header;
cdr << value.joint_names;
cdr << value.points;
}
template<> inline void deserialize(Cdr& cdr, trajectory_msgs::msg::JointTrajectory& value) {
cdr >> value.header;
cdr >> value.joint_names;
cdr >> value.points;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA_CODEC
#define DIMOS_MESSAGE_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.transforms, alignment);
size += calculator.calculate_serialized_size(value.velocities, alignment);
size += calculator.calculate_serialized_size(value.accelerations, alignment);
size += calculator.calculate_serialized_size(value.time_from_start, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& value) {
cdr << value.transforms;
cdr << value.velocities;
cdr << value.accelerations;
cdr << value.time_from_start;
}
template<> inline void deserialize(Cdr& cdr, trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& value) {
cdr >> value.transforms;
cdr >> value.velocities;
cdr >> value.accelerations;
cdr >> value.time_from_start;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2_CODEC
#define DIMOS_MESSAGE_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const trajectory_msgs::msg::MultiDOFJointTrajectory& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.joint_names, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const trajectory_msgs::msg::MultiDOFJointTrajectory& value) {
cdr << value.header;
cdr << value.joint_names;
cdr << value.points;
}
template<> inline void deserialize(Cdr& cdr, trajectory_msgs::msg::MultiDOFJointTrajectory& value) {
cdr >> value.header;
cdr >> value.joint_names;
cdr >> value.points;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E_CODEC
#define DIMOS_MESSAGE_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::BoundingBox2DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.boxes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::BoundingBox2DArray& value) {
cdr << value.header;
cdr << value.boxes;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::BoundingBox2DArray& value) {
cdr >> value.header;
cdr >> value.boxes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65_CODEC
#define DIMOS_MESSAGE_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::BoundingBox3DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.boxes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::BoundingBox3DArray& value) {
cdr << value.header;
cdr << value.boxes;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::BoundingBox3DArray& value) {
cdr >> value.header;
cdr >> value.boxes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A_CODEC
#define DIMOS_MESSAGE_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::ObjectHypothesis& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.class_id, alignment);
size += calculator.calculate_serialized_size(value.score, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::ObjectHypothesis& value) {
cdr << value.class_id;
cdr << value.score;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::ObjectHypothesis& value) {
cdr >> value.class_id;
cdr >> value.score;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339_CODEC
#define DIMOS_MESSAGE_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Classification& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.results, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Classification& value) {
cdr << value.header;
cdr << value.results;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Classification& value) {
cdr >> value.header;
cdr >> value.results;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1_CODEC
#define DIMOS_MESSAGE_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::ObjectHypothesisWithPose& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.hypothesis, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::ObjectHypothesisWithPose& value) {
cdr << value.hypothesis;
cdr << value.pose;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::ObjectHypothesisWithPose& value) {
cdr >> value.hypothesis;
cdr >> value.pose;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29_CODEC
#define DIMOS_MESSAGE_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Detection2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.results, alignment);
size += calculator.calculate_serialized_size(value.bbox, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Detection2D& value) {
cdr << value.header;
cdr << value.results;
cdr << value.bbox;
cdr << value.id;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Detection2D& value) {
cdr >> value.header;
cdr >> value.results;
cdr >> value.bbox;
cdr >> value.id;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A_CODEC
#define DIMOS_MESSAGE_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Detection2DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.detections, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Detection2DArray& value) {
cdr << value.header;
cdr << value.detections;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Detection2DArray& value) {
cdr >> value.header;
cdr >> value.detections;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C_CODEC
#define DIMOS_MESSAGE_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Detection3D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.results, alignment);
size += calculator.calculate_serialized_size(value.bbox, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Detection3D& value) {
cdr << value.header;
cdr << value.results;
cdr << value.bbox;
cdr << value.id;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Detection3D& value) {
cdr >> value.header;
cdr >> value.results;
cdr >> value.bbox;
cdr >> value.id;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE_CODEC
#define DIMOS_MESSAGE_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Detection3DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.detections, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Detection3DArray& value) {
cdr << value.header;
cdr << value.detections;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Detection3DArray& value) {
cdr >> value.header;
cdr >> value.detections;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A_CODEC
#define DIMOS_MESSAGE_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::VisionClass& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.class_id, alignment);
size += calculator.calculate_serialized_size(value.class_name, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::VisionClass& value) {
cdr << value.class_id;
cdr << value.class_name;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::VisionClass& value) {
cdr >> value.class_id;
cdr >> value.class_name;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062_CODEC
#define DIMOS_MESSAGE_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::LabelInfo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.class_map, alignment);
size += calculator.calculate_serialized_size(value.threshold, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::LabelInfo& value) {
cdr << value.header;
cdr << value.class_map;
cdr << value.threshold;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::LabelInfo& value) {
cdr >> value.header;
cdr >> value.class_map;
cdr >> value.threshold;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2_CODEC
#define DIMOS_MESSAGE_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::VisionInfo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.method, alignment);
size += calculator.calculate_serialized_size(value.database_location, alignment);
size += calculator.calculate_serialized_size(value.database_version, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::VisionInfo& value) {
cdr << value.header;
cdr << value.method;
cdr << value.database_location;
cdr << value.database_version;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::VisionInfo& value) {
cdr >> value.header;
cdr >> value.method;
cdr >> value.database_location;
cdr >> value.database_version;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC_CODEC
#define DIMOS_MESSAGE_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::ImageMarker& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.ns, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.action, alignment);
size += calculator.calculate_serialized_size(value.position, alignment);
size += calculator.calculate_serialized_size(value.scale, alignment);
size += calculator.calculate_serialized_size(value.outline_color, alignment);
size += calculator.calculate_serialized_size(value.filled, alignment);
size += calculator.calculate_serialized_size(value.fill_color, alignment);
size += calculator.calculate_serialized_size(value.lifetime, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
size += calculator.calculate_serialized_size(value.outline_colors, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::ImageMarker& value) {
cdr << value.header;
cdr << value.ns;
cdr << value.id;
cdr << value.type;
cdr << value.action;
cdr << value.position;
cdr << value.scale;
cdr << value.outline_color;
cdr << value.filled;
cdr << value.fill_color;
cdr << value.lifetime;
cdr << value.points;
cdr << value.outline_colors;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::ImageMarker& value) {
cdr >> value.header;
cdr >> value.ns;
cdr >> value.id;
cdr >> value.type;
cdr >> value.action;
cdr >> value.position;
cdr >> value.scale;
cdr >> value.outline_color;
cdr >> value.filled;
cdr >> value.fill_color;
cdr >> value.lifetime;
cdr >> value.points;
cdr >> value.outline_colors;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264_CODEC
#define DIMOS_MESSAGE_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::MeshFile& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.filename, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::MeshFile& value) {
cdr << value.filename;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::MeshFile& value) {
cdr >> value.filename;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F_CODEC
#define DIMOS_MESSAGE_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::UVCoordinate& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.u, alignment);
size += calculator.calculate_serialized_size(value.v, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::UVCoordinate& value) {
cdr << value.u;
cdr << value.v;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::UVCoordinate& value) {
cdr >> value.u;
cdr >> value.v;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF_CODEC
#define DIMOS_MESSAGE_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::Marker& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.ns, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.action, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.scale, alignment);
size += calculator.calculate_serialized_size(value.color, alignment);
size += calculator.calculate_serialized_size(value.lifetime, alignment);
size += calculator.calculate_serialized_size(value.frame_locked, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
size += calculator.calculate_serialized_size(value.colors, alignment);
size += calculator.calculate_serialized_size(value.texture_resource, alignment);
size += calculator.calculate_serialized_size(value.texture, alignment);
size += calculator.calculate_serialized_size(value.uv_coordinates, alignment);
size += calculator.calculate_serialized_size(value.text, alignment);
size += calculator.calculate_serialized_size(value.mesh_resource, alignment);
size += calculator.calculate_serialized_size(value.mesh_file, alignment);
size += calculator.calculate_serialized_size(value.mesh_use_embedded_materials, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::Marker& value) {
cdr << value.header;
cdr << value.ns;
cdr << value.id;
cdr << value.type;
cdr << value.action;
cdr << value.pose;
cdr << value.scale;
cdr << value.color;
cdr << value.lifetime;
cdr << value.frame_locked;
cdr << value.points;
cdr << value.colors;
cdr << value.texture_resource;
cdr << value.texture;
cdr << value.uv_coordinates;
cdr << value.text;
cdr << value.mesh_resource;
cdr << value.mesh_file;
cdr << value.mesh_use_embedded_materials;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::Marker& value) {
cdr >> value.header;
cdr >> value.ns;
cdr >> value.id;
cdr >> value.type;
cdr >> value.action;
cdr >> value.pose;
cdr >> value.scale;
cdr >> value.color;
cdr >> value.lifetime;
cdr >> value.frame_locked;
cdr >> value.points;
cdr >> value.colors;
cdr >> value.texture_resource;
cdr >> value.texture;
cdr >> value.uv_coordinates;
cdr >> value.text;
cdr >> value.mesh_resource;
cdr >> value.mesh_file;
cdr >> value.mesh_use_embedded_materials;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95_CODEC
#define DIMOS_MESSAGE_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerControl& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.orientation, alignment);
size += calculator.calculate_serialized_size(value.orientation_mode, alignment);
size += calculator.calculate_serialized_size(value.interaction_mode, alignment);
size += calculator.calculate_serialized_size(value.always_visible, alignment);
size += calculator.calculate_serialized_size(value.markers, alignment);
size += calculator.calculate_serialized_size(value.independent_marker_orientation, alignment);
size += calculator.calculate_serialized_size(value.description, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerControl& value) {
cdr << value.name;
cdr << value.orientation;
cdr << value.orientation_mode;
cdr << value.interaction_mode;
cdr << value.always_visible;
cdr << value.markers;
cdr << value.independent_marker_orientation;
cdr << value.description;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerControl& value) {
cdr >> value.name;
cdr >> value.orientation;
cdr >> value.orientation_mode;
cdr >> value.interaction_mode;
cdr >> value.always_visible;
cdr >> value.markers;
cdr >> value.independent_marker_orientation;
cdr >> value.description;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618_CODEC
#define DIMOS_MESSAGE_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::MenuEntry& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.id, alignment);
size += calculator.calculate_serialized_size(value.parent_id, alignment);
size += calculator.calculate_serialized_size(value.title, alignment);
size += calculator.calculate_serialized_size(value.command, alignment);
size += calculator.calculate_serialized_size(value.command_type, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::MenuEntry& value) {
cdr << value.id;
cdr << value.parent_id;
cdr << value.title;
cdr << value.command;
cdr << value.command_type;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::MenuEntry& value) {
cdr >> value.id;
cdr >> value.parent_id;
cdr >> value.title;
cdr >> value.command;
cdr >> value.command_type;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847_CODEC
#define DIMOS_MESSAGE_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarker& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.description, alignment);
size += calculator.calculate_serialized_size(value.scale, alignment);
size += calculator.calculate_serialized_size(value.menu_entries, alignment);
size += calculator.calculate_serialized_size(value.controls, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarker& value) {
cdr << value.header;
cdr << value.pose;
cdr << value.name;
cdr << value.description;
cdr << value.scale;
cdr << value.menu_entries;
cdr << value.controls;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarker& value) {
cdr >> value.header;
cdr >> value.pose;
cdr >> value.name;
cdr >> value.description;
cdr >> value.scale;
cdr >> value.menu_entries;
cdr >> value.controls;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E_CODEC
#define DIMOS_MESSAGE_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerFeedback& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.client_id, alignment);
size += calculator.calculate_serialized_size(value.marker_name, alignment);
size += calculator.calculate_serialized_size(value.control_name, alignment);
size += calculator.calculate_serialized_size(value.event_type, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.menu_entry_id, alignment);
size += calculator.calculate_serialized_size(value.mouse_point, alignment);
size += calculator.calculate_serialized_size(value.mouse_point_valid, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerFeedback& value) {
cdr << value.header;
cdr << value.client_id;
cdr << value.marker_name;
cdr << value.control_name;
cdr << value.event_type;
cdr << value.pose;
cdr << value.menu_entry_id;
cdr << value.mouse_point;
cdr << value.mouse_point_valid;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerFeedback& value) {
cdr >> value.header;
cdr >> value.client_id;
cdr >> value.marker_name;
cdr >> value.control_name;
cdr >> value.event_type;
cdr >> value.pose;
cdr >> value.menu_entry_id;
cdr >> value.mouse_point;
cdr >> value.mouse_point_valid;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4_CODEC
#define DIMOS_MESSAGE_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerInit& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.server_id, alignment);
size += calculator.calculate_serialized_size(value.seq_num, alignment);
size += calculator.calculate_serialized_size(value.markers, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerInit& value) {
cdr << value.server_id;
cdr << value.seq_num;
cdr << value.markers;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerInit& value) {
cdr >> value.server_id;
cdr >> value.seq_num;
cdr >> value.markers;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5_CODEC
#define DIMOS_MESSAGE_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerPose& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.name, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerPose& value) {
cdr << value.header;
cdr << value.pose;
cdr << value.name;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerPose& value) {
cdr >> value.header;
cdr >> value.pose;
cdr >> value.name;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5_CODEC
#define DIMOS_MESSAGE_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerUpdate& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.server_id, alignment);
size += calculator.calculate_serialized_size(value.seq_num, alignment);
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.markers, alignment);
size += calculator.calculate_serialized_size(value.poses, alignment);
size += calculator.calculate_serialized_size(value.erases, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerUpdate& value) {
cdr << value.server_id;
cdr << value.seq_num;
cdr << value.type;
cdr << value.markers;
cdr << value.poses;
cdr << value.erases;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerUpdate& value) {
cdr >> value.server_id;
cdr >> value.seq_num;
cdr >> value.type;
cdr >> value.markers;
cdr >> value.poses;
cdr >> value.erases;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22_CODEC
#define DIMOS_MESSAGE_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::MarkerArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.markers, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::MarkerArray& value) {
cdr << value.markers;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::MarkerArray& value) {
cdr >> value.markers;
value.validate();
}
#endif
}
