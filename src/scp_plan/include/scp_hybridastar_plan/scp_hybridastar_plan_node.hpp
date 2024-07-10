#ifndef SCP_PLAN_NODE_HPP_
#define SCP_PLAN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/utils.h"

#include "scp_message/msg/agent_action.hpp"
#include "scp_message/msg/agent_action_list.hpp"
#include "scp_message/msg/model_state.hpp"
#include "scp_message/msg/model_state_list.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

#include "json/json.h"
#include "opencv4/opencv2/opencv.hpp"

#include "element.hpp"
#include "type.hpp"
#include "grid_map.hpp"
#include "hybrid_astar.hpp"

using namespace scp;

class SCPHAPlanNode : public rclcpp::Node
{
public:
    SCPHAPlanNode();

public:
    // config
    std::string config_file_path_;
    double scene_width_;
    double scene_height_;
    double position_resolution_;
    int yaw_step_;
    double agent_v;
    double agent_w;
    double agent_dt;
    double check_collision_distance_;
    std::vector<Point> agent_shape_;
    std::vector<Point> agent_anchor_;
    std::vector<Point> good_shape_;
    std::vector<Point> good_anchor_;
    std::vector<Point> wall10_shape_;
    std::vector<Point> wall12_shape_;
    std::vector<Point> obstacle_shape_;

    // Map update
    rclcpp::Subscription<scp_message::msg::ModelStateList>::SharedPtr sub_model_state_list_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_map_;
    scp_message::msg::ModelStateList model_state_list_msg_;
    nav_msgs::msg::OccupancyGrid grid_map_msg_;
    std::vector<Element> static_elements_;
    std::vector<Element> dynamic_elements_;
    Element agent_;
    GridMap grid_map_;

    // Plan 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    geometry_msgs::msg::PoseStamped goal_msg_;
    nav_msgs::msg::Path path_msg_;
    HybridAStar hybrid_astar_;

    // test code
    rclcpp::TimerBase::SharedPtr timer_;
public:
    // config
    void loadConfig();
    void loadModelConfig(std::string model_name, std::vector<Point>* shape, std::vector<Point>* anchor);
    // Map update
    void modelStateListCallback(const scp_message::msg::ModelStateList::SharedPtr msg);
    // Plan
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    // test code
    void timerCallback();
};

#endif // SCP_PLAN_NODE_HPP_