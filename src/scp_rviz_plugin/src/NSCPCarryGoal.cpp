#include "NSCPCarryGoal.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

#include "tf2/utils.h"
#include "opencv2/opencv.hpp"

namespace scp_rviz_plugin
{
NSCPCarryGoal::NSCPCarryGoal()
    : PoseTool2(), qos_profile_(1), count(0)
{
    shortcut_key_ = 'g';

    pub_topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "nscp_carry_goal",
        "The topic on which to publish goals.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    pub_qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        pub_topic_property_, qos_profile_);

    sub_topic_property_ = new rviz_common::properties::StringProperty(
        "Sub Topic", "model_states",
        "The topic on which to subscribe model state list.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    sub_qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        sub_topic_property_, qos_profile_);
}

NSCPCarryGoal::~NSCPCarryGoal() = default;

void NSCPCarryGoal::onInitialize()
{
    PoseTool2::onInitialize();
    arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    pub_qos_profile_property_->initialize(
        [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
    setName("NSCPCarryGoal");
    updateTopic();
}

void NSCPCarryGoal::activate()
{
    PoseTool2::activate();
    //RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: activate");
}

void NSCPCarryGoal::deactivate()
{
    PoseTool2::deactivate();
    //RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: deactivate");
}

int NSCPCarryGoal::onPoseSet2(double x, double y, double theta)
{
    //RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: " << count);
    if (count == 0)
    {
        cv::Point2d point(x, y);
        bool find = false;
        for (auto &state : dynamic_states)
        {
            cv::Point2d state_point(state.pose.position.x, state.pose.position.y);
            if (cv::norm(point - state_point) < 0.5)
            {
                find = true;
                msg.who = state.id;
            }
        }
        if (! find)
        {
            RVIZ_COMMON_LOG_ERROR_STREAM("NSCPCarryGoal: onPoseSet: No model found at (" << x << ", " << y << ")");
            return (Finished | Render);
        }
        else
        {
            RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: onPoseSet: " << msg.who);
            count = 1;
            return (Render);
        }
    }
    else
    {
        msg.going_to.position.x = x;
        msg.going_to.position.y = y;
        msg.going_to.position.z = 0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta);
        msg.going_to.orientation = tf2::toMsg(quat);
        publisher_->publish(msg);
        count = 0;
        logPose("NSCPCarryGoal", msg.going_to.position, msg.going_to.orientation, theta, "map");
        return (Finished | Render);
    }
}

void NSCPCarryGoal::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
    // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
    publisher_ = raw_node->
        template create_publisher<scp_message::msg::ScpCarryTask>(
        pub_topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();

    subscriber_ = raw_node->
        template create_subscription<scp_message::msg::ModelStateList>(
        sub_topic_property_->getStdString(), qos_profile_,
        [this](scp_message::msg::ModelStateList::UniquePtr msg) {
            //RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: updateTopic: " << msg->dynamic_modelstates.size());
            dynamic_states = msg->dynamic_modelstates;
            //count++;
        });

    RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: pubTopic: " << pub_topic_property_->getStdString());
    RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: subTopic: " << sub_topic_property_->getStdString());
}

}  // namespace scp_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(scp_rviz_plugin::NSCPCarryGoal, rviz_common::Tool)