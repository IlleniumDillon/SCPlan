#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/utils.h"

#include "scp_message/msg/agent_action.hpp"
#include "scp_message/msg/agent_action_list.hpp"
#include "scp_message/msg/feedback.hpp"
#include "nav_msgs/msg/path.hpp"

// class PID
// {
// public:
//     double P, I, D;
//     double error, errorSum, errorDiff;
//     double output;
// public:
//     PID(double P, double I, double D)
//         : P(P), I(I), D(D), error(0), errorSum(0), errorDiff(0), output(0)
//     {
//     }
//     void update(double target, double current, double dt)
//     {
//         error = target - current;
//         errorSum += error * dt;
//         errorDiff = (error - errorDiff) / dt;
//         output = P * error + I * errorSum + D * errorDiff;
//     }
//     void update()
//     void reset()
//     {
//         error = 0;
//         errorSum = 0;
//         errorDiff = 0;
//         output = 0;
//     }
// };

class SCPControlNode : public rclcpp::Node
{
public:
    struct parameters
    {
        std::string agent_name;
        double control_frequency;
        double v_max;
        double w_max;
        double K1;
        double K2;
        double K3;
        double K4;
        double K5;
        double K6;
    };
public:
    SCPControlNode();
public:
    // std::string config_path;
    // const std::string config_file = "control_parameter.yaml";

    rclcpp::Subscription<scp_message::msg::AgentActionList>::SharedPtr agentActionListSubscription;
    rclcpp::Subscription<scp_message::msg::Feedback>::SharedPtr agentFeedbackSubscription;
    rclcpp::Publisher<scp_message::msg::AgentAction>::SharedPtr agentActionPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trackPublisher;
    rclcpp::TimerBase::SharedPtr timer;

    scp_message::msg::AgentActionList agentActionList;
    scp_message::msg::AgentAction agentDefaultAction;
    scp_message::msg::Feedback agentFeedback;

    nav_msgs::msg::Path track;

    int index = -1;
    int timeStep = 0;
    bool final = false;

    parameters controlParameters;
private:
    void agentActionCallback(const scp_message::msg::AgentActionList::SharedPtr msg);
    void agentFeedbackCallback(const scp_message::msg::Feedback::SharedPtr msg);
    void timerCallback();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SCPControlNode>());
    rclcpp::shutdown();
    return 0;
}

SCPControlNode::SCPControlNode()
    : Node("scp_control_node")
{
    // try
    // {
    //     config_path = ament_index_cpp::get_package_share_directory("scp_control")
    //         + "/config/";
    // }
    // catch(ament_index_cpp::PackageNotFoundError& e)
    // {
    //     RCLCPP_ERROR(get_logger(), "Package not found: %s", e.what());
    // }
    // RCLCPP_INFO(get_logger(), "Config path: %s", config_path.c_str());

    this->declare_parameter("agent_name", "agent_0");
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("max_velocity", 0.3);
    this->declare_parameter("max_angular_velocity", 0.39269908169872415480783042290994);
    this->declare_parameter("K1", 2.0);
    this->declare_parameter("K2", 1.0);
    this->declare_parameter("K3", 0.0);
    this->declare_parameter("K4", 0.0);
    this->declare_parameter("K5", 0.0);
    this->declare_parameter("K6", 5.0);

    this->get_parameter("agent_name", controlParameters.agent_name);
    this->get_parameter("control_frequency", controlParameters.control_frequency);
    this->get_parameter("max_velocity", controlParameters.v_max);
    this->get_parameter("max_angular_velocity", controlParameters.w_max);
    this->get_parameter("K1", controlParameters.K1);
    this->get_parameter("K2", controlParameters.K2);
    this->get_parameter("K3", controlParameters.K3);
    this->get_parameter("K4", controlParameters.K4);
    this->get_parameter("K5", controlParameters.K5);
    this->get_parameter("K6", controlParameters.K6);

    agentDefaultAction.agent_name = controlParameters.agent_name;
    agentDefaultAction.v = 0;
    agentDefaultAction.w = 0;

    agentActionListSubscription = this->create_subscription<scp_message::msg::AgentActionList>(
        "agent_actions", 1, std::bind(&SCPControlNode::agentActionCallback, this, std::placeholders::_1));
    agentFeedbackSubscription = this->create_subscription<scp_message::msg::Feedback>(
        "agent_feedback", 1, std::bind(&SCPControlNode::agentFeedbackCallback, this, std::placeholders::_1));
    agentActionPublisher = this->create_publisher<scp_message::msg::AgentAction>("agent_action", 1);
    trackPublisher = this->create_publisher<nav_msgs::msg::Path>("track", 1);
    timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000 / controlParameters.control_frequency)), std::bind(&SCPControlNode::timerCallback, this));
}

void SCPControlNode::agentActionCallback(const scp_message::msg::AgentActionList::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received agent action list: %d actions", msg->actions.size());
    agentActionList = *msg;
    index = 0;
    timeStep = 0;
    track.poses.clear();
}

void SCPControlNode::agentFeedbackCallback(const scp_message::msg::Feedback::SharedPtr msg)
{
    agentFeedback = *msg;
    track.header.frame_id = "map";
    track.header.stamp = this->now();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose = msg->pose;
    track.poses.push_back(pose);
    trackPublisher->publish(track);
}

void SCPControlNode::timerCallback()
{
    this->get_parameter("K1", controlParameters.K1);
    this->get_parameter("K2", controlParameters.K2);
    this->get_parameter("K3", controlParameters.K3);
    this->get_parameter("K4", controlParameters.K4);
    this->get_parameter("K5", controlParameters.K5);
    this->get_parameter("K6", controlParameters.K6);
    timeStep++;
    index = timeStep / 25;
    if (index >= 0 && index < agentActionList.actions.size())
    {
        double t = (timeStep % 25) / controlParameters.control_frequency;
        double x1,y1,theta1;
        double x2,y2,theta2;
        x1 = agentActionList.actions[index].came_from.position.x;
        y1 = agentActionList.actions[index].came_from.position.y;
        theta1 = tf2::getYaw(agentActionList.actions[index].came_from.orientation);

        double dth = agentActionList.actions[index].w * t;
        theta2 = theta1 + dth;

        if (abs(dth) < 1e-6)
        {
            x2 = x1 + agentActionList.actions[index].v * t * cos(theta1);
            y2 = y1 + agentActionList.actions[index].v * t * sin(theta1);
        }
        else
        {
            double R = agentActionList.actions[index].v / agentActionList.actions[index].w;
            double Choord = sqrt(2 * R * R * (1 - cos(dth)));
            x2 = x1 + Choord * cos(theta1 + dth / 2);
            y2 = y1 + Choord * sin(theta1 + dth / 2);
        }

        agentDefaultAction.agent_name = controlParameters.agent_name;
        agentDefaultAction.v = agentActionList.actions[index].v;
        agentDefaultAction.w = agentActionList.actions[index].w;
        agentDefaultAction.going_to.position.x = x2;
        agentDefaultAction.going_to.position.y = y2;
        agentDefaultAction.going_to.position.z = 0;
        agentDefaultAction.going_to.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta2));
        agentDefaultAction.final_flag = agentActionList.actions[index].final_flag;
        agentActionPublisher->publish(agentDefaultAction);
    }
    else
    {
        index = -1;
        agentActionPublisher->publish(agentDefaultAction);
    }
    /// @todo Implement the SCP control algorithm
    // scp_message::msg::AgentAction controlOutput = agentDefaultAction;

    // double minDistance = 1e6;
    // int minIndex = -1;
    // for (int i = 0; i < agentActionList.actions.size(); i++)
    // {
    //     double dx = agentActionList.actions[i].going_to.position.x - agentFeedback.pose.position.x;
    //     double dy = agentActionList.actions[i].going_to.position.y - agentFeedback.pose.position.y;
    //     double distance = sqrt(dx * dx + dy * dy);
    //     if (distance <= minDistance)
    //     {
    //         minDistance = distance;
    //         minIndex = i;
    //     }
    // }
    
    // index = minIndex + 1;

    // if (index >= 0 && index < agentActionList.actions.size())
    // {
    //     RCLCPP_INFO(this->get_logger(), "%d", index);
    //     // RCLCPP_INFO(this->get_logger(), "V: %f", agentActionList.actions[index].v);
    //     double dir = agentActionList.actions[index].v > 0 ? 1 : -1;
    //     double dx = agentActionList.actions[index].going_to.position.x - agentFeedback.pose.position.x;
    //     double dy = agentActionList.actions[index].going_to.position.y - agentFeedback.pose.position.y;
    //     tf2::Quaternion q0, q1;
    //     tf2::fromMsg(agentFeedback.pose.orientation, q0);
    //     tf2::fromMsg(agentActionList.actions[index].going_to.orientation, q1);
    //     double dth1 = dir > 0 ? tf2::getYaw(q1) - tf2::getYaw(q0) : tf2::getYaw(q1) - tf2::getYaw(q0) + M_PI;
    //     if (dth1 > M_PI)
    //     {
    //         dth1 -= 2 * M_PI;
    //     }
    //     if (dth1 < -M_PI)
    //     {
    //         dth1 += 2 * M_PI;
    //     }
    //     double dth2 = dir > 0 ? std::atan2(dy, dx) - tf2::getYaw(q0) : std::atan2(dy, dx) - tf2::getYaw(q0) + M_PI;
    //     if (dth2 > M_PI)
    //     {
    //         dth2 -= 2 * M_PI;
    //     }
    //     if (dth2 < -M_PI)
    //     {
    //         dth2 += 2 * M_PI;
    //     }
    //     double distance = std::sqrt(dx * dx + dy * dy);

    //     double v, w;
    //     // double rho = sqrt(dx * dx + dy * dy);
    //     // double alpha = atan2(dy, dx) - tf2::getYaw(q0);
    //     // double beta = -tf2::getYaw(q0) - alpha + tf2::getYaw(q1);

    //     // double v = controlParameters.P * rho;
    //     // double w = controlParameters.P * alpha + controlParameters.P * beta + controlParameters.P * dth;
        
    //     // v = controlParameters.K1 * distance * dir;
    //     v = agentActionList.actions[index].v;
    //     w = agentActionList.actions[index].w;
    //     // if (distance < 0.05)
    //     // {
    //     //     v = 0;
    //     //     w = controlParameters.K6 * dth1;
    //     // }
    //     // else
    //     // {
    //     //     w = controlParameters.K2 * dth2;
    //     // }

    //     if (v > controlParameters.v_max)
    //     {
    //         v = controlParameters.v_max;
    //     }
    //     if (v < -controlParameters.v_max)
    //     {
    //         v = -controlParameters.v_max;
    //     }
    //     if (w > controlParameters.w_max)
    //     {
    //         w = controlParameters.w_max;
    //     }
    //     if (w < -controlParameters.w_max)
    //     {
    //         w = -controlParameters.w_max;
    //     }

    //     controlOutput.v = v;
    //     controlOutput.w = w;

    //     if (agentActionList.actions[index].final_flag)
    //     {
    //         index = -1;
    //     }
    // }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), ".");
    // }

    // agentActionPublisher->publish(controlOutput);
}
