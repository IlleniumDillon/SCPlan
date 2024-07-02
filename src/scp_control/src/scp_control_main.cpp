#include <rclcpp/rclcpp.hpp>

#include "scp_message/msg/agent_action.hpp"
#include "scp_message/msg/agent_action_list.hpp"

class SCPControlNode : public rclcpp::Node
{
public:
    SCPControlNode();
public:
    rclcpp::Subscription<scp_message::msg::AgentActionList>::SharedPtr agentActionListSubscription;
    rclcpp::Publisher<scp_message::msg::AgentAction>::SharedPtr agentActionPublisher;
    rclcpp::TimerBase::SharedPtr timer;

    scp_message::msg::AgentActionList agentActionList;
    scp_message::msg::AgentAction agentDefaultAction;
    int index = -1;
private:
    void agentActionCallback(const scp_message::msg::AgentActionList::SharedPtr msg);
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
    agentDefaultAction.agent_name = "agent_0";
    agentDefaultAction.v = 0;
    agentDefaultAction.w = 0;

    agentActionListSubscription = this->create_subscription<scp_message::msg::AgentActionList>(
        "agent_actions", 1, std::bind(&SCPControlNode::agentActionCallback, this, std::placeholders::_1));
    agentActionPublisher = this->create_publisher<scp_message::msg::AgentAction>("agent_action", 1);
    timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SCPControlNode::timerCallback, this));
}

void SCPControlNode::agentActionCallback(const scp_message::msg::AgentActionList::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received agent action list: %d actions", msg->actions.size());
    agentActionList = *msg;
    index = 0;
}

void SCPControlNode::timerCallback()
{
    if (index >= 0 && index < agentActionList.actions.size())
    {
        agentActionPublisher->publish(agentActionList.actions[index]);
        index++;
    }
    else
    {
        index = -1;
        agentActionPublisher->publish(agentDefaultAction);
    }
}
