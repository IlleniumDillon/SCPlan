#include "InteractiveMove.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

uve_rviz_plugin::InteractiveMove::InteractiveMove()
    : PoseToolBase(), qos_profile_(1)
{
    topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "inter_move_goal",
        "The topic on which to publish goals.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        topic_property_, qos_profile_);
}

uve_rviz_plugin::InteractiveMove::~InteractiveMove() = default;

void uve_rviz_plugin::InteractiveMove::onInitialize()
{
    PoseToolBase::onInitialize();
    qos_profile_property_->initialize(
        [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
    setName("Interactive move");
    updateTopic();
}

void uve_rviz_plugin::InteractiveMove::onPoseSet(double x, double y, double theta)
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

void uve_rviz_plugin::InteractiveMove::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
    // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
    publisher_ = raw_node->
        template create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();
    RVIZ_COMMON_LOG_INFO_STREAM("InteractiveMove: updateTopic: " << topic_property_->getStdString());
}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(uve_rviz_plugin::InteractiveMove, rviz_common::Tool)

