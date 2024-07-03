#include "gazebo_ros_agentaction.hpp"
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <scp_message/msg/agent_action.hpp>
#include <scp_message/msg/feedback.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{

class GazeboRosAgentActionPrivate
{
public:
    void actionCallback(const scp_message::msg::AgentAction::SharedPtr msg);
    void OnUpdate(const gazebo::common::UpdateInfo & info);
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<scp_message::msg::AgentAction>::SharedPtr sub_;
    rclcpp::Publisher<scp_message::msg::Feedback>::SharedPtr pub_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::common::Time last_update_time_;
    double update_period_;
    float v=0, w=0;
    ignition::math::Vector3d position;
    ignition::math::Quaterniond orientation;
    gazebo::physics::ModelPtr pagent = nullptr;
    gazebo::physics::ModelPtr pobject = nullptr;
};

GazeboRosAgentAction::GazeboRosAgentAction():impl_(std::make_unique<GazeboRosAgentActionPrivate>())
{
}

GazeboRosAgentAction::~GazeboRosAgentAction()
{
}

void GazeboRosAgentAction::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
    impl_->world_ = world;
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
    auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
    if (update_rate > 0.0) 
    {
        impl_->update_period_ = 1.0 / update_rate;
    } 
    else 
    {
        impl_->update_period_ = 0.0;
    }
    impl_->sub_ = impl_->ros_node_->create_subscription<scp_message::msg::AgentAction>
        ("/agent_action", qos.get_subscription_qos("agent_action"), std::bind(
            &GazeboRosAgentActionPrivate::actionCallback, impl_.get(), std::placeholders::_1
            )
        );
    impl_->pub_ = impl_->ros_node_->create_publisher<scp_message::msg::Feedback>(
        "/agent_feedback", qos.get_publisher_qos("agent_feedback")
    );
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosAgentActionPrivate::OnUpdate, impl_.get(), std::placeholders::_1)
    );
}

void GazeboRosAgentActionPrivate::actionCallback(const scp_message::msg::AgentAction::SharedPtr msg)
{
    pagent = world_->ModelByName(msg->agent_name);
    pobject = world_->ModelByName(msg->object_name);
    if (pagent == nullptr)
    {
        RCLCPP_WARN(ros_node_->get_logger(), "Agent %s not found", msg->agent_name.c_str());
        return;
    }

    v = msg->v;
    w = msg->w;
    position = ignition::math::Vector3d(
        msg->going_to.position.x, msg->going_to.position.y, msg->going_to.position.z
    );
    orientation = ignition::math::Quaterniond(
        msg->going_to.orientation.w, msg->going_to.orientation.x,
        msg->going_to.orientation.y, msg->going_to.orientation.z
    );
    
    /// XXX: CHECK IF THIS IS THE RIGHT WAY TO DO THIS
    if (msg->action != 0 && pobject != nullptr)
    {
        gazebo::physics::LinkPtr linkAgent = pagent->GetLink("link_0");
        gazebo::physics::LinkPtr linkObject = pobject->GetLink("link_0");
        if (linkAgent == nullptr || linkObject == nullptr)
        {
            RCLCPP_WARN(ros_node_->get_logger(), "Link not found");
            return;
        }
        if (msg->action == 1)
        {
            pagent->CreateJoint("joint_0", "fixed", linkAgent, linkObject);
        }
        else if (msg->action == -1)
        {
            pagent->RemoveJoint("joint_0");
        }
    }

    //last_update_time_ = world_->SimTime();
}

void GazeboRosAgentActionPrivate::OnUpdate(const gazebo::common::UpdateInfo &info)
{
    if (pagent == nullptr)
    {
        return;
    }
    gazebo::common::Time current_time = info.simTime;
    if (current_time < last_update_time_) {
        RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
        last_update_time_ = current_time;
    }
    double seconds_since_last_update = (current_time - last_update_time_).Double();
    if (seconds_since_last_update < update_period_) {
        return;
    }
    last_update_time_ = current_time;
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("GazeboRosAgentActionPrivate::OnUpdate");
    IGN_PROFILE_BEGIN("update");
#endif
    // double currentYaw = pagent->WorldPose().Rot().Yaw();
    // ignition::math::Vector3d V = ignition::math::Vector3d(
    //     v * cos(currentYaw), v * sin(currentYaw), 0
    // );
    // pagent->SetLinearVel(V);
    // pagent->SetAngularVel(ignition::math::Vector3d(0, 0, w));

    // double dt = seconds_since_last_update;
    // ignition::math::Quaterniond q = pagent->WorldPose().Rot();
    // double theta0 = q.Yaw();
    // ignition::math::Vector3d position0 = pagent->WorldPose().Pos();
    // ignition::math::Vector3d position1;
    // double dtheta = w * dt;
    // double theta1 = theta0 + dtheta;
    // if (abs(dtheta) < 1e-6)
    // {
    //     double dx = v * dt * std::cos(theta0);
    //     double dy = v * dt * std::sin(theta0);

    //     position1.X() = position0.X() + dx;
    //     position1.Y() = position0.Y() + dy;
    // }
    // else
    // {
    //     double R = v / w;
    //     double Choord = std::sqrt(
    //         2 * R * R * (1 - std::cos(dtheta))
    //     );
    //     double dx = Choord * std::cos(theta0 + dtheta / 2);
    //     double dy = Choord * std::sin(theta0 + dtheta / 2);

    //     position1.X() = position0.X() + dx;
    //     position1.Y() = position0.Y() + dy;
    // }
    // pagent->SetWorldPose(
    //     ignition::math::Pose3d(
    //         position1, ignition::math::Quaterniond(0, 0, theta1)
    //     )
    // );
    // scp_message::msg::Feedback feedback;
    // feedback.pose.position.x = position0.X();
    // feedback.pose.position.y = position0.Y();
    // feedback.pose.position.z = position0.Z();
    // feedback.pose.orientation.x = q.X();
    // feedback.pose.orientation.y = q.Y();
    // feedback.pose.orientation.z = q.Z();
    // feedback.pose.orientation.w = q.W();
    // pub_->publish(feedback);

    pagent->SetWorldPose(
        ignition::math::Pose3d(position, orientation)
    );
    scp_message::msg::Feedback feedback;
    feedback.pose.position.x = position.X();
    feedback.pose.position.y = position.Y();
    feedback.pose.position.z = position.Z();
    feedback.pose.orientation.x = orientation.X();
    feedback.pose.orientation.y = orientation.Y();
    feedback.pose.orientation.z = orientation.Z();
    feedback.pose.orientation.w = orientation.W();
    pub_->publish(feedback);
    
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosAgentAction)

}