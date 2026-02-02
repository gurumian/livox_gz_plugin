#ifndef LIVOX_GZ_PLUGIN__LIVOX_POINTS_PLUGIN_HPP_
#define LIVOX_GZ_PLUGIN__LIVOX_POINTS_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs.hh>
#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace livox_gz_plugin
{
class LivoxPointsPlugin : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPostUpdate
{
public:
  LivoxPointsPlugin();
  ~LivoxPointsPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  void OnScan(const gz::msgs::PointCloudPacked &_msg);

  gz::transport::Node gz_node_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  std::string scan_topic_;
  std::string ros_topic_;
  std::string frame_id_;
};
}  // namespace livox_gz_plugin

#endif  // LIVOX_GZ_PLUGIN__LIVOX_POINTS_PLUGIN_HPP_
