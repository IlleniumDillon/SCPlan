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
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;
    float v=0, w=0;
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
    impl_->sub_ = impl_->ros_node_->create_subscription<scp_message::msg::AgentAction>
        ("/agent_action", qos.get_subscription_qos("agent_action"), std::bind(
            &GazeboRosAgentActionPrivate::actionCallback, impl_.get(), std::placeholders::_1
            )
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
}

void GazeboRosAgentActionPrivate::OnUpdate(const gazebo::common::UpdateInfo &info)
{
    if (pagent == nullptr)
    {
        return;
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("GazeboRosAgentActionPrivate::OnUpdate");
    IGN_PROFILE_BEGIN("update");
#endif
    double currentYaw = pagent->WorldPose().Rot().Yaw();
    ignition::math::Vector3d V = ignition::math::Vector3d(
        v * cos(currentYaw), v * sin(currentYaw), 0
    );
    pagent->SetLinearVel(V);
    pagent->SetAngularVel(ignition::math::Vector3d(0, 0, w));  
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosAgentAction)

}