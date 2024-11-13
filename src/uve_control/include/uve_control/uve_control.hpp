#ifndef UVE_CONTROL_HPP
#define UVE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "uvs_tools/ctrl/mpc.hpp"
#include "uvs_message/msg/uv_emb_arm.hpp"
#include "uvs_message/msg/uv_emb_emag.hpp"
#include "uvs_message/msg/uv_emb_kinetics.hpp"
#include "uve_message/action/uve_path_track.hpp"
#include "uve_message/msg/uve_agent_status.hpp"

#include <thread>
#include <future>

class UveControl : public rclcpp::Node
{
public:
    UveControl();
    ~UveControl();

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const uve_message::action::UvePathTrack::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<uve_message::action::UvePathTrack>> goal_handle);
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<uve_message::action::UvePathTrack>> goal_handle); 
    
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uve_message::action::UvePathTrack>> goal_handle);

    void status_callback(const uve_message::msg::UveAgentStatus::SharedPtr msg);

    void timer_callback();

private:
    rclcpp_action::Server<uve_message::action::UvePathTrack>::SharedPtr action_server_;
    rclcpp::Publisher<uvs_message::msg::UvEmbArm>::SharedPtr pub_arm_;
    rclcpp::Publisher<uvs_message::msg::UvEmbEmag>::SharedPtr pub_emag_;
    rclcpp::Publisher<uvs_message::msg::UvEmbKinetics>::SharedPtr pub_kinetics_;
    rclcpp::Subscription<uve_message::msg::UveAgentStatus>::SharedPtr sub_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<MPC> mpc_;
    double mpc_rate = 0;

    std::shared_future<void> future_;

    uve_message::msg::UveAgentStatus status_;
};

#endif // UVE_CONTROL_HPP