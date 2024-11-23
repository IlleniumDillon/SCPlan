#include <rclcpp/rclcpp.hpp>

#include "uvs_message/msg/uv_opt_pose_list.hpp"
#include "tf2/utils.h"

class UveSimOptitrackNode : public rclcpp::Node
{
public:
    UveSimOptitrackNode()
        : Node("uve_simoptitrack")
    {
        declare_parameter("sim_opt_name");
        declare_parameter("sim_opt_pose");
        auto name = get_parameter("sim_opt_name").as_string_array();
        auto pose = get_parameter("sim_opt_pose").as_double_array();
        for (int i = 0; i < name.size(); i++)
        {
            uvs_message::msg::UvOptPose pose_item;
            pose_item.name = name[i];
            pose_item.pose.position.x = pose[i * 3];
            pose_item.pose.position.y = pose[i * 3 + 1];
            pose_item.pose.position.z = 0;
            pose_item.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), pose[i * 3 + 2]));
            pose_list_.pose_list.push_back(pose_item);
        }
        pub_pose_list_ = create_publisher<uvs_message::msg::UvOptPoseList>("uvs_pose_list", 1);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UveSimOptitrackNode::timer_callback, this));
    }
    ~UveSimOptitrackNode() = default;
private:
    void timer_callback()
    {
        pub_pose_list_->publish(pose_list_);
    }

private:
    rclcpp::Publisher<uvs_message::msg::UvOptPoseList>::SharedPtr pub_pose_list_;
    rclcpp::TimerBase::SharedPtr timer_;
    uvs_message::msg::UvOptPoseList pose_list_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UveSimOptitrackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}