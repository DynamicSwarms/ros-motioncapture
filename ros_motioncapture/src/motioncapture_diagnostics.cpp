#include "ros_motioncapture/motioncapture_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ros_motioncapture {
namespace {
double threshold(const DiagnosticsNodeInterfaces & interfaces, const std::string & name, double value,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  value = interfaces.parameters->declare_parameter(
    name, rclcpp::ParameterValue(value), descriptor).get<double>();
  if (!std::isfinite(value) || value < 0) {
    throw std::invalid_argument(name + " must be finite and nonnegative");
  }
  return value;
}
}  // namespace

MotionCaptureDiagnostics::MotionCaptureDiagnostics(DiagnosticsNodeInterfaces interfaces)
: m_interfaces(std::move(interfaces)),
  m_latency_threshold(threshold(m_interfaces, "latency_threshold", 0.035,
    "Total SDK latency warning threshold in seconds; 0 disables"))
{
  m_publisher = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    m_interfaces.parameters, m_interfaces.topics, "/diagnostics", rclcpp::QoS(10));
}

void CaptureStatistics::record_frame(const rclcpp::Time & stamp,
  const std::vector<libmotioncapture::LatencyInfo> & latency,
  size_t marker_count, size_t body_count)
{
  received = stamp;
  ++frames;
  ++interval_frames;
  markers = marker_count;
  bodies = body_count;
  latencies = latency;
  total_latency = 0;
  for (const auto & item : latency) {
    if (!std::isfinite(item.value()) || item.value() < 0) invalid_latency = true;
    else total_latency += item.value();
  }
  peak_latency = std::max(peak_latency, total_latency);
}

void CaptureStatistics::reset_interval()
{
  interval_frames = 0;
  peak_latency = 0;
  invalid_latency = false;
}

void MotionCaptureDiagnostics::record_consumed(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  ++m_consumed;
  m_last_publish_delay = std::max(0.0, (m_interfaces.clock->get_clock()->now() - stamp).seconds());
  m_max_publish_delay = std::max(m_max_publish_delay, m_last_publish_delay);
}

void MotionCaptureDiagnostics::report_error(const std::string & message)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_error = message;
  RCLCPP_ERROR(m_interfaces.logging->get_logger(), "Motion capture failed: %s", message.c_str());
}

void MotionCaptureDiagnostics::publish(bool connected, const CaptureStatistics & capture)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  const auto now = Clock::now();
  const double age = capture.frames ?
    std::max(0.0, (m_interfaces.clock->get_clock()->now() - capture.received).seconds()) :
    std::chrono::duration<double>(now - m_created).count();
  const double interval = std::chrono::duration<double>(now - m_interval).count();
  Status status;
  status.name = std::string(m_interfaces.base->get_fully_qualified_name()) + "/motioncapture";
  status.hardware_id = m_interfaces.parameters->get_parameter("type").as_string() + ":" +
    m_interfaces.parameters->get_parameter("hostname").as_string();
  status.message = connected ? (capture.frames ? "Running" : "Waiting for first frame") : "Connecting";
  const std::string initial_message = status.message;
  const auto raise = [&status, &initial_message](uint8_t level, const std::string & message) {
      status.level = std::max(status.level, level);
      if (status.message == initial_message) {
        status.message = message;
      } else {status.message += "; " + message;}
    };
  const bool connection_delayed = !connected && m_error.empty() &&
    now - m_created >= std::chrono::seconds(1);
  if (connection_delayed) {
    raise(Status::ERROR, "Motion capture not connected after 1 second");
    if (!m_connectionErrorReported) {
      RCLCPP_ERROR(m_interfaces.logging->get_logger(),
        "Motion capture not connected after 1 second");
      m_connectionErrorReported = true;
    }
  }
  if (!m_error.empty()) raise(Status::ERROR, m_error);
  if (capture.invalid_latency) raise(Status::WARN, "Invalid SDK latency in interval");
  if (m_latency_threshold > 0 && capture.peak_latency > m_latency_threshold) {
    raise(Status::WARN, "Motion capture latency high");
    RCLCPP_WARN(m_interfaces.logging->get_logger(),
      "MoCap latency high: peak %.4f s (threshold %.4f s); see /diagnostics for components",
      capture.peak_latency, m_latency_threshold);
  }
  const auto add = [&status](const std::string & key, auto value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      status.values.push_back(std::move(item));
    };
  add("Connected", connected);
  add("Frames total", capture.frames);
  add("Consumed frames total", m_consumed);
  add("Overwritten frames total", capture.overwritten);
  add("Last acquisition-to-publish delay (s)", m_last_publish_delay);
  add("Maximum acquisition-to-publish delay in interval (s)", m_max_publish_delay);
  add("Frame rate (Hz)", interval > 0 ? capture.interval_frames / interval : 0);
  add("Frame received", capture.frames > 0);
  add("Last frame age / startup wait (s)", age);
  add("Markers in last frame", capture.markers);
  add("Rigid bodies in last frame", capture.bodies);
  add("Latency available in last frame", !capture.latencies.empty());
  add("Total latency in last frame (s)", capture.total_latency);
  add("Maximum total latency in interval (s)", capture.peak_latency);
  for (const auto & latency : capture.latencies) {
    add("Latency: " + latency.name() + " (s)", latency.value());
  }
  add("Latency warning threshold (s; 0 disables)", m_latency_threshold);
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = m_interfaces.clock->get_clock()->now();
  message.status.push_back(std::move(status));
  m_publisher->publish(message);
  m_interval = now;
  m_max_publish_delay = 0;
}
}  // namespace ros_motioncapture
