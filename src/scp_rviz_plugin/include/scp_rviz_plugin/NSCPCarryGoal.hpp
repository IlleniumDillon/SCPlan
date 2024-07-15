#ifndef NSCP_CARRY_GOAL_HPP
#define NSCP_CARRY_GOAL_HPP

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

// #include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "_pose_tool2.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "scp_message/msg/scp_carry_task.hpp"
#include "scp_message/msg/model_state_list.hpp"

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace scp_rviz_plugin
{
class NSCPCarryGoal : public PoseTool2
{
    Q_OBJECT
public:
  NSCPCarryGoal();

  ~NSCPCarryGoal() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  
protected:
  int onPoseSet2(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<scp_message::msg::ScpCarryTask>::SharedPtr publisher_;
  rclcpp::Subscription<scp_message::msg::ModelStateList>::SharedPtr subscriber_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty * pub_topic_property_;
  rviz_common::properties::QosProfileProperty * pub_qos_profile_property_;

  rviz_common::properties::StringProperty * sub_topic_property_;
  rviz_common::properties::QosProfileProperty * sub_qos_profile_property_;

  rclcpp::QoS qos_profile_;

  int count;
  std::vector<scp_message::msg::ModelState> dynamic_states;
  scp_message::msg::ScpCarryTask msg;
};
} // namespace scp_rviz_plugin

#endif // NSCP_CARRY_GOAL_HPP