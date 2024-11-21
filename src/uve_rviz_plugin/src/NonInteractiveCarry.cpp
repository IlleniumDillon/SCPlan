#include "NonInteractiveCarry.hpp"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

#include "tf2/utils.h"
#include "opencv2/opencv.hpp"

using namespace uve_rviz_plugin;

NonInteractiveCarry::NonInteractiveCarry()
    : rviz_common::Tool(), arrow_(nullptr), angle_(0),
    qos_profile_(1), count(0)
{
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();

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

NonInteractiveCarry::~NonInteractiveCarry() = default;

void NonInteractiveCarry::onInitialize()
{
    arrow_ = std::make_shared<rviz_rendering::Arrow>(
        scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
    arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);

    pub_qos_profile_property_->initialize(
        [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
    setName("NSCPCarryGoal");
    updateTopic();
}

void NonInteractiveCarry::activate()
{
    setStatus("Click and drag mouse to set position/orientation.");
    state_ = Position;
}

void NonInteractiveCarry::deactivate()
{
    arrow_->getSceneNode()->setVisible(false);
}

int NonInteractiveCarry::processMouseEvent(rviz_common::ViewportMouseEvent &event)
{
    auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

    if (event.leftDown()) 
    {
        return processMouseLeftButtonPressed(point_projection_on_xy_plane);
    } 
    else if (event.type == QEvent::MouseMove && event.left()) 
    {
        return processMouseMoved(point_projection_on_xy_plane);
    } 
    else if (event.leftUp()) 
    {
        return processMouseLeftButtonReleased();
    }

    return 0;
}

int uve_rviz_plugin::NonInteractiveCarry::onPoseSet(double x, double y, double theta)
{
    if (count == 0)
    {
        cv::Point2d point(x, y);
        bool find = false;
        for (auto &state : status_list_.list)
        {
            cv::Point2d state_point(state.pose.x, state.pose.y);
            if (cv::norm(point - state_point) < 0.5)
            {
                find = true;
                goal_.name = state.name;
            }
        }
        if (! find)
        {
            RVIZ_COMMON_LOG_ERROR_STREAM("NSCPCarryGoal: onPoseSet: No model found at (" << x << ", " << y << ")");
            return (Finished | Render);
        }
        else
        {
            RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: onPoseSet: " << goal_.name);
            count = 1;
            return (Render);
        }
    }
    else
    {
        goal_.goal.x = x;
        goal_.goal.y = y;
        goal_.goal.theta = theta;
        publisher_->publish(goal_);
        count = 0;
        logPose("NSCPCarryGoal", x, y, theta, "map");
        return (Finished | Render);
    }
}

geometry_msgs::msg::Quaternion NonInteractiveCarry::orientationAroundZAxis(double angle)
{
    auto orientation = geometry_msgs::msg::Quaternion();
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = sin(angle) / (2 * cos(angle / 2));
    orientation.w = cos(angle / 2);
    return orientation;
}

void uve_rviz_plugin::NonInteractiveCarry::logPose(std::string designation, double x, double y, double angle, std::string frame)
{
    RVIZ_COMMON_LOG_INFO_STREAM(
    "Setting " << designation << " pose: Frame:" << frame << ", Position(" << x << ", " <<
      y << "), Angle: " << angle);
}

// void NonInteractiveCarry::logPose(std::string designation, geometry_msgs::msg::Point position, geometry_msgs::msg::Quaternion orientation, double angle, std::string frame)
// {
//     RVIZ_COMMON_LOG_INFO_STREAM(
//     "Setting " << designation << " pose: Frame:" << frame << ", Position(" << position.x << ", " <<
//       position.y << ", " << position.z << "), Orientation(" << orientation.x << ", " <<
//       orientation.y << ", " << orientation.z << ", " << orientation.w <<
//       ") = Angle: " << angle);
// }

int NonInteractiveCarry::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
    int flags = 0;
    assert(state_ == Position);
    if (xy_plane_intersection.first) 
    {
        arrow_position_ = xy_plane_intersection.second;
        arrow_->setPosition(arrow_position_);

        state_ = Orientation;
        flags |= Render;
    }
    return flags;
}

int NonInteractiveCarry::processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
    int flags = 0;
    if (state_ == Orientation) 
    {
        // compute angle in x-y plane
        if (xy_plane_intersection.first) 
        {
            angle_ = calculateAngle(xy_plane_intersection.second, arrow_position_);
            makeArrowVisibleAndSetOrientation(angle_);

            flags |= Render;
        }
    }

    return flags;
}

int NonInteractiveCarry::processMouseLeftButtonReleased()
{
    int flags = 0;
    if (state_ == Orientation) 
    {
        flags |= onPoseSet(arrow_position_.x, arrow_position_.y, angle_);
        state_ = Position;
    }

    return flags;
}

void NonInteractiveCarry::makeArrowVisibleAndSetOrientation(double angle)
{
    arrow_->getSceneNode()->setVisible(true);

    // we need base_orient, since the arrow goes along the -z axis by default
    // (for historical reasons)
    Ogre::Quaternion orient_x = Ogre::Quaternion(
        Ogre::Radian(-Ogre::Math::HALF_PI),
        Ogre::Vector3::UNIT_Y);

    arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
}

double NonInteractiveCarry::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point)
{
    return atan2(start_point.y - end_point.y, start_point.x - end_point.x);
}

void uve_rviz_plugin::NonInteractiveCarry::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
    // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
    publisher_ = raw_node->
        template create_publisher<uve_message::msg::NonInteractiveCarryGoal>(
        pub_topic_property_->getStdString(), qos_profile_);
    clock_ = raw_node->get_clock();

    subscriber_ = raw_node->
        template create_subscription<uve_message::msg::UveDynamicStatusList>(
        sub_topic_property_->getStdString(), qos_profile_,
        [this](uve_message::msg::UveDynamicStatusList::UniquePtr msg) {
            //RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: updateTopic: " << msg->dynamic_modelstates.size());
            status_list_ = *msg;
            //count++;
        });

    RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: pubTopic: " << pub_topic_property_->getStdString());
    RVIZ_COMMON_LOG_INFO_STREAM("NSCPCarryGoal: subTopic: " << sub_topic_property_->getStdString());
}


#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(uve_rviz_plugin::NonInteractiveCarry, rviz_common::Tool)
