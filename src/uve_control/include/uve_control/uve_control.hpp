#ifndef UVE_CONTROL_HPP
#define UVE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "uvs_tools/ctrl/mpc.hpp"
#include "uvs_message/msg/uv_emb_arm.hpp"
#include "uvs_message/msg/uv_emb_emag.hpp"
#include "uvs_message/msg/uv_emb_kinetics.hpp"
#include "uvs_message/msg/uv_emb_status.hpp"
// #include "uve_message/action/uve_path_track.hpp"
#include "uve_message/msg/uve_plan_result.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/utils.h"
// #include "uve_message/msg/uve_agent_status.hpp"

#include <thread>
#include <future>
#include <atomic>

class UveControl : public rclcpp::Node
{
public:
    UveControl();
    ~UveControl();

private:
    void status_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);

    void emb_callback(const uvs_message::msg::UvEmbStatus::SharedPtr msg);

    void control_ref_callback(const uve_message::msg::UvePlanResult::SharedPtr msg);
    void control_abort_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void controlTask();
    bool _taskAlive()
    {
        if (rclcpp::ok() && !abortFlag.load())
        {
            return true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Task Abort");
        }
        return false;
    }

private:
    // rclcpp_action::Server<uve_message::action::UvePathTrack>::SharedPtr action_server_;
    rclcpp::Publisher<uvs_message::msg::UvEmbArm>::SharedPtr pub_arm_;
    rclcpp::Publisher<uvs_message::msg::UvEmbEmag>::SharedPtr pub_emag_;
    rclcpp::Publisher<uvs_message::msg::UvEmbKinetics>::SharedPtr pub_kinetics_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_status_;
    rclcpp::Subscription<uvs_message::msg::UvEmbStatus>::SharedPtr sub_emb_;

    rclcpp::Subscription<uve_message::msg::UvePlanResult>::SharedPtr control_ref_sub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr control_get_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr control_finish_pub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr control_abort_sub;

    std::shared_ptr<MPC> mpc_;
    double mpc_rate = 0;
    double wheelWidth = 0.0;

    std::shared_future<void> future_;

    geometry_msgs::msg::Pose2D status_;
    uvs_message::msg::UvEmbStatus emb_;

    uve_message::msg::UvePlanResult path;
    std::atomic<bool> abortFlag;

    int arm_arm_off;
    int arm_arm_on;
    int arm_base_off;
    int arm_base_on;
};

#endif // UVE_CONTROL_HPP