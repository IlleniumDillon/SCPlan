#include "NSCPMoveGoal.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

namespace scp_rviz_plugin
{
NSCPMoveGoal::NSCPMoveGoal()
    : rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
    shortcut_key_ = 'g';

    topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "nscp_goal",
        "The topic on which to publish goals.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        topic_property_, qos_profile_);
}

NSCPMoveGoal::~NSCPMoveGoal() = default;

void NSCPMoveGoal::onInitialize()
{
    PoseTool::onInitialize();
    qos_profile_property_->initialize(
        [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
    setName("NSCPMoveGoal");
    updateTopic();
}

void NSCPMoveGoal::onPoseSet(double x, double y, double theta)
{
   std::string fixed_frame = context_->getFixedFrame().toStdString();

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = clock_->now();
    goal.header.frame_id = fixed_frame;

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    goal.pose.orientation = orientationAroundZAxis(theta);

    logPose("goal", goal.pose.position, goal.pose.orientation, theta, fixed_frame);

    publisher_->publish(goal); 
}

void NSCPMoveGoal::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
    // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
    publisher_ = raw_node->
        template create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();
    RVIZ_COMMON_LOG_INFO_STREAM("NSCPMoveGoal: updateTopic: " << topic_property_->getStdString());
}

}  // namespace scp_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(scp_rviz_plugin::NSCPMoveGoal, rviz_common::Tool)