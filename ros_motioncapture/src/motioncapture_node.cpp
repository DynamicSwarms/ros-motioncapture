#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>
#include <stdexcept>
#include <vector>

#include "ros_motioncapture/motioncapture_diagnostics.hpp"

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

std::set<std::string> extract_names(
  const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
  const std::string& pattern)
{
  std::set<std::string> result;
  for (const auto &i : parameter_overrides)
  {
    if (i.first.find(pattern) == 0)
    {
      size_t start = pattern.size() + 1;
      size_t end = i.first.find(".", start);
      result.insert(i.first.substr(start, end - start));
    }
  }
  return result;
}

std::vector<double> get_vec(const rclcpp::ParameterValue& param_value)
{
  if (param_value.get_type() == rclcpp::PARAMETER_INTEGER_ARRAY) {
    const auto int_vec = param_value.get<std::vector<int64_t>>();
    std::vector<double> result;
    for (int v : int_vec) {
      result.push_back(v);
    }
    return result;
  }
  return param_value.get<std::vector<double>>();
}

class MotionCaptureNode : public rclcpp::Node
{
public:
  MotionCaptureNode()
  : rclcpp::Node("motioncapture_node")
  , tfbroadcaster(rclcpp::node_interfaces::NodeInterfaces(this->get_node_parameters_interface(), this->get_node_topics_interface()))
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    std::string motionCaptureType = declare_parameter<std::string>("type", "vicon", descriptor);
    m_cfg["hostname"] = declare_parameter<std::string>("hostname", "localhost", descriptor);
    m_cfg["add_labeled_markers_to_pointcloud"] = declare_parameter<bool>("add_labeled_markers_to_pointcloud", true, descriptor) ? "true" : "false";
    const auto topicName = declare_parameter<std::string>("topic_name", "pointCloud", descriptor);
    m_frameId = declare_parameter<std::string>("frame_id", "world", descriptor);


    diagnostics = std::make_unique<ros_motioncapture::MotionCaptureDiagnostics>(
      ros_motioncapture::DiagnosticsNodeInterfaces{
        get_node_base_interface(), get_node_clock_interface(),
        get_node_parameters_interface(), get_node_topics_interface(),
        get_node_logging_interface()});
    m_diagnosticsTimer = create_wall_timer(std::chrono::seconds(1),
      std::bind(&MotionCaptureNode::publishDiagnostics, this));

    if (motionCaptureType == "mock") initializeMockRigidBodies();

    initPointCloud();
    pubPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(topicName, 1);

    RCLCPP_INFO(this->get_logger(), "Connecting to %s on '%s'.", m_cfg.at("hostname").c_str(), motionCaptureType.c_str());
    try {
      m_publishingThread = std::thread(&MotionCaptureNode::publishingLoop, this);
      m_acquisitionThread = std::thread(acquisitionLoop, m_captureState,
        std::move(motionCaptureType), m_cfg, get_clock());
    } catch (...) {
      stop();
      throw;
    }
  }

  ~MotionCaptureNode() override {stop();}

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(m_captureState->mutex);
      m_captureState->stop.store(true);
    }
    m_captureState->changed.notify_all();

    // Publishing uses the node, so it must finish before ROS resources are destroyed.
    if (m_publishingThread.joinable()) m_publishingThread.join();

    if (!m_acquisitionThread.joinable()) return;
    if (m_captureState->connected.load()) m_acquisitionThread.join();
    else m_acquisitionThread.detach();

  }

private:
  void publishDiagnostics()
  {
    ros_motioncapture::CaptureStatistics capture;
    {
      std::lock_guard<std::mutex> lock(m_captureState->mutex);
      capture = m_captureState->statistics;
      m_captureState->statistics.reset_interval();
    }
    diagnostics->publish(m_captureState->connected.load(), capture);
  }

  struct CaptureResult
  {
    rclcpp::Time stamp;
    libmotioncapture::PointCloud pointcloud;
    std::map<std::string, libmotioncapture::RigidBody> bodies;
    std::string error;
  };

  struct CaptureState
  {
    std::atomic<bool> stop{false};
    std::mutex mutex;
    std::condition_variable changed;
    std::atomic<bool> connected{false};
    std::unique_ptr<CaptureResult> pending;
    ros_motioncapture::CaptureStatistics statistics;
  };

  // May outlive the node: owns its state, SDK client, and clock; never accesses the node or publishers.
  static void acquisitionLoop(
    std::shared_ptr<CaptureState> state,
    std::string type,
    std::map<std::string, std::string> config,
    rclcpp::Clock::SharedPtr clock)
  {
    try {
      std::unique_ptr<libmotioncapture::MotionCapture> client(libmotioncapture::MotionCapture::connect(type, config));
      if (!client) throw std::runtime_error("Motion capture connection returned no client");
      state->connected.store(true);

      while (!state->stop.load()) {
        client->waitForNextFrame();
        auto stamp = clock->now();

        auto result = std::make_unique<CaptureResult>();
        result->stamp = stamp;
        result->pointcloud = client->pointCloud();
        result->bodies = client->rigidBodies();
        const auto & latencies = client->latency();

        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->statistics.record_frame(stamp, latencies, result->pointcloud.rows(), result->bodies.size());
          if (state->pending) ++state->statistics.overwritten;
          state->pending = std::move(result);
        }
        state->changed.notify_all();
      }

    } catch (const std::exception & error) {
      auto result = std::make_unique<CaptureResult>();
      result->error = error.what();
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->pending) ++state->statistics.overwritten;
      state->pending = std::move(result);
    }

    state->changed.notify_all();
  }

  std::unique_ptr<CaptureResult> waitForResult()
  {
    std::unique_lock<std::mutex> lock(m_captureState->mutex);
    while (!m_captureState->stop.load() && !m_captureState->pending) {
      m_captureState->changed.wait(lock);
    }
    if (m_captureState->stop.load()) return nullptr;
    return std::move(m_captureState->pending);
  }

  void publishingLoop()
  {
    try {
      while (true) {
        auto result = waitForResult();
        if (!result || !rclcpp::ok()) return;
        if (!result->error.empty()) {
          diagnostics->report_error(result->error);
        } else {
          publishFrame(*result);
        }
      }
    } catch (const std::exception & error) {
      diagnostics->report_error(error.what());
    }
  }

void publishFrame(const CaptureResult & frame)
{
  const auto & time = frame.stamp;
  const auto & pointcloud = frame.pointcloud;

  diagnostics->record_consumed(frame.stamp);

  // publish as pointcloud
  msgPointCloud.header.stamp = time;
  msgPointCloud.width = pointcloud.rows();
  msgPointCloud.data.resize(pointcloud.rows() * 3 * 4); // width * height * pointstep
  memcpy(msgPointCloud.data.data(), pointcloud.data(), msgPointCloud.data.size());
  msgPointCloud.row_step = msgPointCloud.data.size();

  pubPointCloud->publish(msgPointCloud);

  transforms.clear();
  transforms.reserve(frame.bodies.size());
  for (const auto & body : frame.bodies)
  {
    const auto & rigidBody = body.second;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = rigidBody.position().cast<double>();

    const auto & quaternion = rigidBody.rotation();
    if (quaternion.coeffs().allFinite() && quaternion.coeffs().isUnitary())
    {
      transform.linear() = quaternion.cast<double>().normalized().toRotationMatrix();
    } // Invalid orientations retain the identity rotation.

    auto message = tf2::eigenToTransform(transform);
    message.header.stamp = time;
    message.header.frame_id = m_frameId;
    message.child_frame_id = rigidBody.name();

    transforms.push_back(std::move(message));
  }

  if (!transforms.empty()) {
    tfbroadcaster.sendTransform(transforms);
  }
}

  void initializeMockRigidBodies()
  {
    const auto & parameter_overrides = get_node_parameters_interface()->get_parameter_overrides();
    for (const auto & name : extract_names(parameter_overrides, "rigid_bodies")) {
      const auto position = get_vec(parameter_overrides.at("rigid_bodies." + name + ".initial_position"));
      m_cfg["rigid_bodies"] += name + "(" + std::to_string(position[0]) + "," +
        std::to_string(position[1]) + "," + std::to_string(position[2]) + ",1,0,0,0);";
    }
  }

  void initPointCloud()
  {
    msgPointCloud.header.frame_id = m_frameId;
    msgPointCloud.height = 1;

    sensor_msgs::msg::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    msgPointCloud.fields.push_back(field);
    field.name = "y";
    field.offset = 4;
    msgPointCloud.fields.push_back(field);
    field.name = "z";
    field.offset = 8;
    msgPointCloud.fields.push_back(field);
    msgPointCloud.point_step = 12;
    msgPointCloud.is_bigendian = false;
    msgPointCloud.is_dense = true;
  }

  std::unique_ptr<ros_motioncapture::MotionCaptureDiagnostics> diagnostics;
  rclcpp::TimerBase::SharedPtr m_diagnosticsTimer;
  std::shared_ptr<CaptureState> m_captureState{std::make_shared<CaptureState>()};
  std::thread m_acquisitionThread;
  std::thread m_publishingThread;

  std::map<std::string, std::string> m_cfg;
  std::string m_frameId;
  sensor_msgs::msg::PointCloud2 msgPointCloud;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubPointCloud;

  tf2_ros::TransformBroadcaster tfbroadcaster;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotionCaptureNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  node->stop();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
