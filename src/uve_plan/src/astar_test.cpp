#include <rclcpp/rclcpp.hpp>
#include "astar/astar.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp" 
#include "nav_msgs/msg/path.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/utils.h"

class AStarTestNode : public rclcpp::Node
{
public:
    AStarTestNode() : Node("astar_test")
    {
        sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, std::bind(&AStarTestNode::map_callback, this, std::placeholders::_1));
        sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, std::bind(&AStarTestNode::goal_callback, this, std::placeholders::_1));
        pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 1);
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
    }
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (getPoseNum == 0)
        {
            start_.x = msg->pose.position.x;
            start_.y = msg->pose.position.y;
            start_.theta = tf2::getYaw(msg->pose.orientation);
            getPoseNum++;
        }
        else
        {
            goal_.x = msg->pose.position.x;
            goal_.y = msg->pose.position.y;
            goal_.theta = tf2::getYaw(msg->pose.orientation);
            getPoseNum = 0;

            Eigen::Matrix<int, 1, 2> size;
            size << map_.info.width, map_.info.height;
            Eigen::Matrix<double, 1, 2> resolution;
            resolution << map_.info.resolution, map_.info.resolution;
            Eigen::Matrix<double, 1, 2> origin;
            origin << map_.info.origin.position.x, map_.info.origin.position.y;
            AStarGraph graph(size, resolution, origin);
            std::cout << graph.grid_size << std::endl;
            for (int i = 0; i < map_.info.width; i++)
            {
                for (int j = 0; j < map_.info.height; j++)
                {
                    auto node = graph[{i, j}];
                    node->occupied = map_.data[j * map_.info.width + i] > 50;
                    // std::cout << node->occupied;
                }
                // std::cout << std::endl;
            }
            AStarSearch search;
            search.updateGraph(graph);
            auto start = graph[{(int)((start_.x - origin[0]) / resolution[0]), (int)((start_.y - origin[1]) / resolution[1])}];
            std::cout << "start: " << start->state << std::endl;
            auto goal = graph[{(int)((goal_.x - origin[0]) / resolution[0]), (int)((goal_.y - origin[1]) / resolution[1])}];
            std::cout << "goal: " << goal->state << std::endl;
            search.result = search.search(start, goal);
            std::cout << "success: " << search.result.success << std::endl;
            std::cout << "time: " << search.result.planTime / 10e+9 << std::endl;
            std::cout << "it: " << search.result.iterations << std::endl;
            if (search.result.success)
            {
                nav_msgs::msg::Path path;
                path.header.frame_id = "map";
                for (auto& trace : search.result.trace)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.pose.position.x = trace[0];
                    pose.pose.position.y = trace[1];
                    path.poses.push_back(pose);
                }
                pub_path_->publish(path);
            }
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::Pose2D start_;
    geometry_msgs::msg::Pose2D goal_;
    int getPoseNum = 0;
};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarTestNode>());
    rclcpp::shutdown();
    return 0;
}
