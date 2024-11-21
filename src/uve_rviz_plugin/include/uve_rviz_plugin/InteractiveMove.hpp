#ifndef INTERACTIVE_MOVE_HPP
#define INTERACTIVE_MOVE_HPP

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "PoseToolBase.hpp"

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
class InteractiveMove : public PoseToolBase
{
    Q_OBJECT
public:
    InteractiveMove();
    ~InteractiveMove() override;
    void onInitialize() override;
protected:
    void onPoseSet(double x, double y, double theta) override;
private Q_SLOTS:
    void updateTopic();
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;

    rviz_common::properties::StringProperty * topic_property_;
    rviz_common::properties::QosProfileProperty * qos_profile_property_;

    rclcpp::QoS qos_profile_;
};
}  // namespace uve_rviz_plugin

#endif // INTERACTIVE_MOVE_HPP