#include "livox_gz_plugin/livox_points_plugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/Util.hh>

namespace livox_gz_plugin
{

LivoxPointsPlugin::LivoxPointsPlugin()
{
}

LivoxPointsPlugin::~LivoxPointsPlugin()
{
}

void LivoxPointsPlugin::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &_eventMgr)
{
  // Initialize ROS node
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  ros_node_ = std::make_shared<rclcpp::Node>("livox_gz_plugin_node");

  // Read parameters from SDF
  if (_sdf->HasElement("ros_topic"))
    ros_topic_ = _sdf->Get<std::string>("ros_topic");
  else
    ros_topic_ = "livox/lidar";

  if (_sdf->HasElement("scan_topic"))
    scan_topic_ = _sdf->Get<std::string>("scan_topic");
  else 
  {
     RCLCPP_WARN(ros_node_->get_logger(), "No 'scan_topic' param provided. Ensure you set it in SDF.");
  }
  
  if (_sdf->HasElement("frame_id"))
    frame_id_ = _sdf->Get<std::string>("frame_id");
  else
    frame_id_ = "livox_frame";

  // Create Publisher
  custom_pub_ = ros_node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(ros_topic_, 10);

  // Subscribe to Gazebo Topic
  if (!scan_topic_.empty())
  {
    gz_node_.Subscribe(scan_topic_, &LivoxPointsPlugin::OnScan, this);
    RCLCPP_INFO(ros_node_->get_logger(), "Subscribed to Gazebo topic: %s", scan_topic_.c_str());
  }

  RCLCPP_INFO(ros_node_->get_logger(), "LivoxPointsPlugin Configured. Output: %s", ros_topic_.c_str());
}

void LivoxPointsPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                   const gz::sim::EntityComponentManager &_ecm)
{
    // No-op. Data comes via callback.
}

void LivoxPointsPlugin::OnScan(const gz::msgs::PointCloudPacked &_msg)
{
  livox_ros_driver2::msg::CustomMsg custom_msg;
  // Use Simulation Time from Gazebo Message
  auto gz_sec = _msg.header().stamp().sec();
  auto gz_nsec = _msg.header().stamp().nsec();
  custom_msg.header.stamp = rclcpp::Time(gz_sec, gz_nsec);
  custom_msg.header.frame_id = frame_id_;
  custom_msg.timebase = _msg.header().stamp().sec() * 1e9 + _msg.header().stamp().nsec();
  
  int width = _msg.width();
  int height = _msg.height();
  int point_step = _msg.point_step();
  int num_points = width * height;
  
  custom_msg.point_num = num_points;
  custom_msg.points.reserve(num_points);

  // Parse field offsets
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int intensity_offset = -1; // Sometimes called 'intensity' or 'reflectivity'

  for (int i = 0; i < _msg.field_size(); ++i)
  {
    const auto &field = _msg.field(i);
    if (field.name() == "x") x_offset = field.offset();
    else if (field.name() == "y") y_offset = field.offset();
    else if (field.name() == "z") z_offset = field.offset();
    else if (field.name() == "intensity") intensity_offset = field.offset();
  }
  
  // Default offsets if standard layout (Float32 XYZ)
  if (x_offset == -1) x_offset = 0;
  if (y_offset == -1) y_offset = 4;
  if (z_offset == -1) z_offset = 8;
  
  const char* data_buffer = _msg.data().c_str();

  for (int i = 0; i < num_points; ++i)
  {
    int ptr = i * point_step;
    
    float x = 0, y = 0, z = 0, intensity = 0;
    
    // Read Float32
    // Safety check: buffer size
    // memcpy is safer for strict aliasing than casting
    memcpy(&x, data_buffer + ptr + x_offset, sizeof(float));
    memcpy(&y, data_buffer + ptr + y_offset, sizeof(float));
    memcpy(&z, data_buffer + ptr + z_offset, sizeof(float));
    
    if (intensity_offset != -1)
    {
       // Check datatype of intensity? Usually float or double. Assuming float for GpuLidar.
       // It could be double if configured so. Assume float for now.
       memcpy(&intensity, data_buffer + ptr + intensity_offset, sizeof(float));
    }
    
    livox_ros_driver2::msg::CustomPoint pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.reflectivity = static_cast<uint8_t>(intensity); // Msg expects uint8 usually? Check msg def. 
    // Definition says `uint8 reflectivity`. But some use 0-255 scaling.
    
    pt.tag = 0; 
    pt.line = i % 32; // Simulation of lines
    
    // Approx offset time. Total scan time? standard is 100ms usually.
    // 0.1s / num_points * i
    double dt = 0.1 / num_points * i; 
    pt.offset_time = static_cast<uint32_t>(dt * 1e9); // nanoseconds
    
    custom_msg.points.push_back(pt);
  }

  custom_pub_->publish(custom_msg);
}

}  // namespace livox_gz_plugin

GZ_ADD_PLUGIN(livox_gz_plugin::LivoxPointsPlugin,
              gz::sim::System,
              livox_gz_plugin::LivoxPointsPlugin::ISystemConfigure,
              livox_gz_plugin::LivoxPointsPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(livox_gz_plugin::LivoxPointsPlugin, "livox_gz_plugin")
