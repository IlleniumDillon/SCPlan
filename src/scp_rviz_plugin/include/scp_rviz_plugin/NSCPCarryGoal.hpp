#ifndef NSCP_CARRY_GOAL_HPP
#define NSCP_CARRY_GOAL_HPP

#include <OgreVector3.h>

#include <QCursor>  // NOLINT cpplint cannot handle the include order here
#include <QObject>  // NOLINT cpplint cannot handle the include order here

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_common/tool.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class StringProperty;
class BoolProperty;
class QosProfileProperty;
}
}

namespace scp_rviz_plugin
{
class NSCPCarryGoal : public rviz_common::Tool
{
    Q_OBJECT
public:
    NSCPCarryGoal();

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;

private Q_SLOTS:
    void updateTopic();
    void updateAutoDeactivate();
protected:
    void publishPosition(const Ogre::Vector3 & position) const;
    void setStatusForPosition(const Ogre::Vector3 & position);

    QCursor std_cursor_;
    QCursor hit_cursor_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;

    rviz_common::properties::StringProperty * topic_property_;
    rviz_common::properties::BoolProperty * auto_deactivate_property_;
    rviz_common::properties::QosProfileProperty * qos_profile_property_;

    rclcpp::QoS qos_profile_;
};
} // namespace scp_rviz_plugin

#endif // NSCP_CARRY_GOAL_HPP