#include <iostream>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

void logWarn(rclcpp::Logger logger, const std::string& msg)
{
  RCLCPP_WARN(logger, "%s", msg.c_str());
}

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
    this->declare_parameter<std::string>("type", "vicon");
    this->declare_parameter<std::string>("hostname", "localhost");
    this->declare_parameter<std::string>("topic_name", "pointCloud");
    this->declare_parameter<bool>("add_labeled_markers_to_pointcloud", true);
    this->declare_parameter<double>("latency_threshold", 0.035);

    std::string motionCaptureType = this->get_parameter("type").as_string();
    std::string motionCaptureHostname = this->get_parameter("hostname").as_string();
    std::string topicName = this->get_parameter("topic_name").as_string();
    bool addLabeledMarkersToPointCloud = this->get_parameter("add_labeled_markers_to_pointcloud").as_bool();

    auto node_parameters_iface = this->get_node_parameters_interface();
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
    node_parameters_iface->get_parameter_overrides();


    std::stringstream sstr;
    sstr << "Connecting to " << motionCaptureHostname << " on '" << motionCaptureType << "'. Might block indefinetely if not reachable.";
    logWarn(this->get_logger(), sstr.str());
    // Make a new client
    std::map<std::string, std::string> cfg;
    cfg["hostname"] = motionCaptureHostname;
    cfg["add_labeled_markers_to_pointcloud"] = addLabeledMarkersToPointCloud ? "true" : "false";

    // if the mock type is selected, add the defined rigid bodies
    if (motionCaptureType == "mock") {
      auto rigid_body_names = extract_names(parameter_overrides, "rigid_bodies");
      for (const auto &name : rigid_body_names)
      {
        const auto pos = get_vec(parameter_overrides.at("rigid_bodies." + name + ".initial_position"));
        cfg["rigid_bodies"] += name + "(" + std::to_string(pos[0]) + "," + std::to_string(pos[1]) + "," + std::to_string(pos[2]) +",1,0,0,0);";
      }
    }

    mocap.reset(libmotioncapture::MotionCapture::connect(motionCaptureType, cfg));    
    msgPointCloud.header.frame_id = "world";
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
    
    // prepare point cloud publisher
    pubPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(topicName, 1);
    
    m_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MotionCaptureNode::timer_callback, this));
}

void timer_callback() 
{
  mocap->waitForNextFrame();

  auto time = this->now();
  auto pointcloud = mocap->pointCloud();

  const auto& mocapLatency = mocap->latency();
  float totalMocapLatency = 0;
  for (const auto& item : mocapLatency) {
    totalMocapLatency += item.value();
  }
  if (totalMocapLatency > this->get_parameter("latency_threshold").as_double()) {
    std::stringstream sstr;
    sstr << "MoCap Latency high: " << totalMocapLatency << " s.";
    logWarn(this->get_logger(), sstr.str());
    for (const auto& item : mocapLatency) {
      std::stringstream sstr;
      sstr << "  Latency: " << item.name() << ": " << item.value() << " s.";
      logWarn(this->get_logger(), sstr.str());
    }
  }

  // publish as pointcloud
  msgPointCloud.header.stamp = time;
  msgPointCloud.width = pointcloud.rows();
  msgPointCloud.data.resize(pointcloud.rows() * 3 * 4); // width * height * pointstep
  memcpy(msgPointCloud.data.data(), pointcloud.data(), msgPointCloud.data.size());
  msgPointCloud.row_step = msgPointCloud.data.size();

  pubPointCloud->publish(msgPointCloud);
  
  transforms.clear();
  transforms.reserve(mocap->rigidBodies().size());
  for (const auto &iter : mocap->rigidBodies())
  {
    const auto& rigidBody = iter.second;

    // const auto& transform = rigidBody.transformation();
    // transforms.emplace_back(eigenToTransform(transform));
    transforms.resize(transforms.size() + 1);
    transforms.back().header.stamp = time;
    transforms.back().header.frame_id = "world";
    transforms.back().child_frame_id = rigidBody.name();
    transforms.back().transform.translation.x = rigidBody.position().x();
    transforms.back().transform.translation.y = rigidBody.position().y();
    transforms.back().transform.translation.z = rigidBody.position().z();
    transforms.back().transform.rotation.x = rigidBody.rotation().x();
    transforms.back().transform.rotation.y = rigidBody.rotation().y();
    transforms.back().transform.rotation.z = rigidBody.rotation().z();
    transforms.back().transform.rotation.w = rigidBody.rotation().w();
  }

  if (transforms.size() > 0) {
    // send TF. Since RViz and others can't handle nan's, report a fake oriention if needed
    for (auto& tf : transforms) {
      if (std::isnan(tf.transform.rotation.x)) {
        tf.transform.rotation.x = 0;
        tf.transform.rotation.y = 0;
        tf.transform.rotation.z = 0;
        tf.transform.rotation.w = 1;
      }
    }

    tfbroadcaster.sendTransform(transforms);
  }
}


private: 
  std::unique_ptr<libmotioncapture::MotionCapture> mocap;

  sensor_msgs::msg::PointCloud2 msgPointCloud;

  int m_current_frame = -1;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubPointCloud;
  std::shared_ptr<rclcpp::TimerBase> m_timer;

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
  rclcpp::shutdown();
  return 0;  
}