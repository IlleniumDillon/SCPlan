#include "scp_rviz_moveto.hpp"

scp_rviz_moveto::MoveTo::MoveTo() : qos_profile_(5)
{
    shortcut_key_ = 'm';

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

void scp_rviz_moveto::MoveTo::onInitialize()
{
    hit_cursor_ = cursor_;
    std_cursor_ = rviz_common::getDefaultCursor();
    qos_profile_property_->initialize(
        [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
    updateTopic();
}

void scp_rviz_moveto::MoveTo::activate()
{
}

void scp_rviz_moveto::MoveTo::deactivate()
{
}

void scp_rviz_moveto::MoveTo::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
    // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
    publisher_ = raw_node->
        template create_publisher<geometry_msgs::msg::PointStamped>(
        topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();
}

int scp_rviz_moveto::MoveTo::processMouseEvent(rviz_common::ViewportMouseEvent &event)
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

void scp_rviz_moveto::MoveTo::updateAutoDeactivate()
{
}

void scp_rviz_moveto::MoveTo::publishPosition(const Ogre::Vector3 &position)
{
    std::ostringstream s;
    s << "<b>Left-Click:</b> Select this point.";
    s.precision(3);
    s << " [" << position.x << "," << position.y << "," << position.z << "]";
    this->setStatus(s.str().c_str());
}

void scp_rviz_moveto::MoveTo::setStatusForPosition(const Ogre::Vector3 &position)
{
    auto point = rviz_common::pointOgreToMsg(position);
    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.point = point;
    point_stamped.header.frame_id = context_->getFixedFrame().toStdString();
    point_stamped.header.stamp = clock_->now();
    publisher_->publish(point_stamped);
}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(scp_rviz_moveto::MoveTo, rviz_common::Tool)