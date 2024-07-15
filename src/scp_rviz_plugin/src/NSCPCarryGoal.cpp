#include "NSCPCarryGoal.hpp"

#include <sstream>

#include <OgreVector3.h>

#include "rclcpp/qos.hpp"

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

namespace scp_rviz_plugin
{
NSCPCarryGoal::NSCPCarryGoal() : qos_profile_(5)
{
    shortcut_key_ = 'u';

    topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "/clicked_point",
        "The topic on which to publish points.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    auto_deactivate_property_ = new rviz_common::properties::BoolProperty(
        "Single click", true,
        "Switch away from this tool after one click.",
        getPropertyContainer(), SLOT(updateAutoDeactivate()), this);

    qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        topic_property_, qos_profile_);
}

void NSCPCarryGoal::onInitialize()
{
    hit_cursor_ = cursor_;
    std_cursor_ = rviz_common::getDefaultCursor();
    qos_profile_property_->initialize(
        [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
    updateTopic();
}

void NSCPCarryGoal::activate()
{
}

void NSCPCarryGoal::deactivate()
{
}

int NSCPCarryGoal::processMouseEvent(rviz_common::ViewportMouseEvent &event)
{
    int flags = 0;

    Ogre::Vector3 position;
    bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position);
    setCursor(success ? hit_cursor_ : std_cursor_);

    if (success) {
        setStatusForPosition(position);

        if (event.leftUp()) {
        publishPosition(position);

        if (auto_deactivate_property_->getBool()) {
            flags |= Finished;
        }
        }
    } else {
        setStatus("Move over an object to select the target point.");
    }

    return flags;
}

void NSCPCarryGoal::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
    // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
    publisher_ = raw_node->
        template create_publisher<geometry_msgs::msg::PointStamped>(
        topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();
}

void NSCPCarryGoal::updateAutoDeactivate()
{
}

void NSCPCarryGoal::publishPosition(const Ogre::Vector3 &position) const
{
    auto point = rviz_common::pointOgreToMsg(position);
    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.point = point;
    point_stamped.header.frame_id = context_->getFixedFrame().toStdString();
    point_stamped.header.stamp = clock_->now();
    publisher_->publish(point_stamped);
}

void NSCPCarryGoal::setStatusForPosition(const Ogre::Vector3 &position)
{
    std::ostringstream s;
    s << "<b>Left-Click:</b> Select this point.";
    s.precision(3);
    s << " [" << position.x << "," << position.y << "," << position.z << "]";
    setStatus(s.str().c_str());
}

}  // namespace scp_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(scp_rviz_plugin::NSCPCarryGoal, rviz_common::Tool)