#include "gazebo_ros_modelstate.hpp"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros_modelstate.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <scp_message/msg/model_state_list.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosModelStatePrivate
{
public:
    void OnUpdate(const gazebo::common::UpdateInfo & info);
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<scp_message::msg::ModelStateList>::SharedPtr pub_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    double update_period_;
    gazebo::common::Time last_update_time_;
    std::mutex lock_;
    gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosModelState::GazeboRosModelState():impl_(std::make_unique<GazeboRosModelStatePrivate>())
{
}

GazeboRosModelState::~GazeboRosModelState()
{
}
void GazeboRosModelState::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
    impl_->world_ = world;
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
    auto update_rate = sdf->Get<double>("update_rate", 10.0).first;
    if (update_rate > 0.0) 
    {
        impl_->update_period_ = 1.0 / update_rate;
    } 
    else 
    {
        impl_->update_period_ = 0.0;
    }
    impl_->last_update_time_ = impl_->world_->SimTime();
    impl_->pub_ = impl_->ros_node_->create_publisher<scp_message::msg::ModelStateList>(
        "model_states",qos.get_publisher_qos("model_states", rclcpp::QoS(1))
    );
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosModelStatePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosModelStatePrivate::OnUpdate(const gazebo::common::UpdateInfo &info)
{
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
    IGN_PROFILE("GazeboRosModelStatePrivate::OnUpdate");
    IGN_PROFILE_BEGIN("update");
#endif
    std::lock_guard<std::mutex> scoped_lock(lock_);
    gazebo::physics::Model_V modelList = world_->Models();
    scp_message::msg::ModelStateList msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
    msg.header.frame_id = "map";
    for(auto pmodel : modelList)
    {
        scp_message::msg::ModelState state;
        state.name = pmodel->GetName();
        ignition::math::Pose3d modelPose = pmodel->RelativePose();
        ignition::math::Quaterniond modelRot = modelPose.Rot();
        state.pose.position.x = modelPose.X();
        state.pose.position.y = modelPose.Y();
        state.pose.position.z = modelPose.Z();
        state.pose.orientation.x = modelRot.X();
        state.pose.orientation.y = modelRot.Y();
        state.pose.orientation.z = modelRot.Z();
        state.pose.orientation.w = modelRot.W();
        msg.modelstates.push_back(state);
    }
    pub_->publish(msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosModelState)

}