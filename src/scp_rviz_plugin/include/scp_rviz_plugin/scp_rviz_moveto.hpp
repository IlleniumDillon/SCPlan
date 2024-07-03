#ifndef SCP_RVIZ_PLUGIN__SCP_RVIZ_MOVETO_HPP_
#define SCP_RVIZ_PLUGIN__SCP_RVIZ_MOVETO_HPP_

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <QCursor>  // NOLINT cpplint cannot handle the include order here

#include "rviz_common/tool.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/view_controller.hpp"


namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace scp_rviz_moveto
{
class RVIZ_DEFAULT_PLUGINS_PUBLIC MoveTo : public rviz_common::Tool
{
    Q_OBJECT
public:
    MoveTo();
    void onInitialize() override;

    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

public Q_SLOTS:

    void updateTopic();
    void updateAutoDeactivate();

protected:
    void publishPosition(const Ogre::Vector3 & position);
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
}  // namespace scp_rviz_moveto

#endif // SCP_RVIZ_PLUGIN__SCP_RVIZ_MOVETO_HPP_