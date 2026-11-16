#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <libmotioncapture/motioncapture.h>
#include <rclcpp/rclcpp.hpp>

namespace ros_motioncapture {

struct DiagnosticsNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging;
};

// Plain acquisition data; guarded by the capture state's mutex, with no node ownership.
struct CaptureStatistics
{
  size_t frames{0}, interval_frames{0}, overwritten{0}, markers{0}, bodies{0};
  rclcpp::Time received{0, 0, RCL_ROS_TIME};
  std::vector<libmotioncapture::LatencyInfo> latencies;
  double total_latency{0}, peak_latency{0};
  bool invalid_latency{false};

  void record_frame(const rclcpp::Time & stamp,
    const std::vector<libmotioncapture::LatencyInfo> & latency,
    size_t marker_count, size_t body_count);
  void reset_interval();
};

// Reports copied frame statistics without accessing the SDK.
class MotionCaptureDiagnostics
{
public:
  explicit MotionCaptureDiagnostics(DiagnosticsNodeInterfaces interfaces);
  void publish(bool connected, const CaptureStatistics & capture);
  void record_consumed(const rclcpp::Time & stamp);
  void report_error(const std::string & message);

private:
  using Clock = std::chrono::steady_clock;
  DiagnosticsNodeInterfaces m_interfaces;
  double m_latency_threshold;
  std::mutex m_mutex;
  Clock::time_point m_created{Clock::now()}, m_interval{m_created};
  size_t m_consumed{0};
  double m_last_publish_delay{0}, m_max_publish_delay{0};
  bool m_connectionErrorReported{false};
  std::string m_error;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_publisher;
};
}  // namespace ros_motioncapture
