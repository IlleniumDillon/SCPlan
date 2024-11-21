#ifndef NON_INTERACTIVE_CARRY_HPP
#define NON_INTERACTIVE_CARRY_HPP

#include <memory>
#include <string>
#include <utility>

#include <OgreVector3.h>

#include <QCursor>  // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "uve_message/msg/non_interactive_carry_goal.hpp"
#include "uve_message/msg/uve_dynamic_status_list.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

namespace rviz_rendering
{
class Arrow;
}  // namespace rviz_rendering

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace uve_rviz_plugin
{
class NonInteractiveCarry : public rviz_common::Tool
{
    Q_OBJECT
public:
  NonInteractiveCarry();

  ~NonInteractiveCarry() override;

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  int onPoseSet(double x, double y, double theta);

  geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle);

  void logPose(
    std::string designation,
    double x,
    double y,
    double angle,
    std::string frame);

  std::shared_ptr<rviz_rendering::Arrow> arrow_;

  enum State
  {
    Position,
    Orientation
  };
  State state_;
  double angle_;

  Ogre::Vector3 arrow_position_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

private:
  int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseLeftButtonReleased();
  void makeArrowVisibleAndSetOrientation(double angle);
  double calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point);

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<uve_message::msg::NonInteractiveCarryGoal>::SharedPtr publisher_;
  rclcpp::Subscription<uve_message::msg::UveDynamicStatusList>::SharedPtr subscriber_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty * pub_topic_property_;
  rviz_common::properties::QosProfileProperty * pub_qos_profile_property_;

  rviz_common::properties::StringProperty * sub_topic_property_;
  rviz_common::properties::QosProfileProperty * sub_qos_profile_property_;

  rclcpp::QoS qos_profile_;

  int count;

  uve_message::msg::NonInteractiveCarryGoal goal_;
  uve_message::msg::UveDynamicStatusList status_list_;
};
}  // namespace uve_rviz_plugin


#endif // NON_INTERACTIVE_CARRY_HPP