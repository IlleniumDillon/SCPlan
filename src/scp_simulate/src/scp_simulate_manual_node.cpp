#include <rclcpp/rclcpp.hpp>
#include "scp_message/msg/agent_action.hpp"
#include <thread>
#include <future>
#include <atomic>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
 
#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "
 
int get_char();
 
int get_char()
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;
 
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 1000; //设置等待超时时间
 
    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0){
        ch = getchar(); 
    }
    fflush(stdin);
    return ch;
}

class SCPManualSimulate : public rclcpp::Node
{
public:
    SCPManualSimulate(): Node("scp_simulate_manual")
    {
        publisher = this->create_publisher<scp_message::msg::AgentAction>("/agent_action", 10);
    }
public:
    rclcpp::Publisher<scp_message::msg::AgentAction>::SharedPtr publisher;
    bool runnning = true;
};

int main(int argc, char * argv[])
{
    // system(STTY_US TTY_PATH);
    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<SCPManualSimulate>();
    // rclcpp::spin(node);
    // rclcpp::on_shutdown(std::bind(&SCPManualSimulate::stop, node));
    // rclcpp::shutdown();
    // system(STTY_DEF TTY_PATH);
    int ch = 0;
    system(STTY_US TTY_PATH);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SCPManualSimulate>();

    while(rclcpp::ok())
    {
        ch = get_char();
        // RCLCPP_INFO(node->get_logger(), "Key: %c \n", ch);
        if (ch == ' ')
        {
            system(STTY_DEF TTY_PATH);
            break;
        }
        scp_message::msg::AgentAction msg;
        msg.agent_name = "agent_0";
        msg.object_name = "";
        msg.action = 0;
        switch (ch)
        {
        case 'q':
            msg.v = 1;
            msg.w = 1;
            break;
        case 'w':
            msg.v = 1;
            msg.w = 0;
            break;
        case 'e':
            msg.v = 1;
            msg.w = -1;
            break;
        case 'a':
            msg.v = 0;
            msg.w = 1;
            break;
        case 's':
            msg.v = 0;
            msg.w = 0;
            break;
        case 'd':
            msg.v = 0;
            msg.w = -1;
            break;
        case 'z':
            msg.v = -1;
            msg.w = 1;
            break;
        case 'x':
            msg.v = -1;
            msg.w = 0;
            break;
        case 'c':
            msg.v = -1;
            msg.w = -1;
            break;
        default:
            msg.v = 0;
            msg.w = 0;
            break;
        }
        node->publisher->publish(msg);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    system(STTY_DEF TTY_PATH);
    rclcpp::shutdown();
    return 0;
}