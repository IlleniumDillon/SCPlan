#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "hybrid_astar/hybrid_astar.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp" 
#include "nav_msgs/msg/path.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "uve_message/msg/uve_agent_status.hpp"
#include "uve_message/action/uve_path_track.hpp"

class HybridAStarTestNode : public rclcpp::Node
{
public:
    HybridAStarTestNode() : Node("hybrid_astar_test")
    {
        sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, std::bind(&HybridAStarTestNode::map_callback, this, std::placeholders::_1));
        sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, std::bind(&HybridAStarTestNode::goal_callback, this, std::placeholders::_1));
        sub_status_ = create_subscription<geometry_msgs::msg::Pose2D>("uve_agent_status", 1, std::bind(&HybridAStarTestNode::status_callback, this, std::placeholders::_1));
        pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 1);
        pub_trace_ = create_publisher<geometry_msgs::msg::PoseArray>("trace", 1);
        action_client_ = rclcpp_action::create_client<uve_message::action::UvePathTrack>(this, "uve_path_track");
        RCLCPP_INFO(get_logger(), "HybridAStarTestNode has been started.");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
    }
    void status_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        start_.x = msg->x;
        start_.y = msg->y;
        start_.theta = msg->theta;
    }
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        goal_.x = msg->pose.position.x;
        goal_.y = msg->pose.position.y;
        goal_.theta = tf2::getYaw(msg->pose.orientation);
        if (goal_.theta < 0)
        {
            goal_.theta += 2 * M_PI;
        }

        Eigen::Matrix<int, 1, 3> size;
        size << map_.info.width, map_.info.height , 32;
        Eigen::Matrix<double, 1, 3> resolution;
        resolution << map_.info.resolution, map_.info.resolution ,M_PI / 8;
        Eigen::Matrix<double, 1, 3> origin;
        origin << map_.info.origin.position.x, map_.info.origin.position.y , 0.0;
        HybridAStarGraph graph(size, resolution, origin);
        std::cout << graph.grid_size << std::endl;
        for (int i = 0; i < map_.info.width; i++)
        {
            for (int j = 0; j < map_.info.height; j++)
            {
                bool occ = map_.data[j * map_.info.width + i] > 50;
                for (int k = 0; k < 32; k++)
                {
                    auto node = graph({i, j, k});
                    node->occupied = occ;
                }
                // std::cout << node->occupied;
            }
            // std::cout << std::endl;
        }
        std::cout << "map loaded" << std::endl;
        HybridAStarSearch search;
        search.generateNeighborList(0.3, M_PI / 4, 2, 2, 0.5);
        std::cout << "neighbor list generated" << std::endl;
        search.updateGraph(graph);
        std::cout << "graph updated" << std::endl;
        auto start = graph({(int)((start_.x - origin[0]) / resolution[0]), (int)((start_.y - origin[1]) / resolution[1]), (int)(start_.theta / resolution[2])});
        if (start == nullptr)
        {
            std::cout << "start is null" << std::endl;
            return;
        }
        std::cout << "start: " << start->state << std::endl;
        auto goal = graph({(int)((goal_.x - origin[0]) / resolution[0]), (int)((goal_.y - origin[1]) / resolution[1]), (int)(goal_.theta / resolution[2])});
        if (goal == nullptr)
        {
            std::cout << "goal is null" << std::endl;
            return;
        }
        std::cout << "goal: " << goal->state << std::endl;
        search.result = search.search(start, goal);
        std::cout << "success: " << search.result.success << std::endl;
        std::cout << "time: " << search.result.planTime / 10e+9 << std::endl;
        std::cout << "it: " << search.result.iterations << std::endl;
        if (search.result.success)
        {
            geometry_msgs::msg::PoseArray t;
            nav_msgs::msg::Path path;
            path.header.frame_id = "map";
            t.header.frame_id = "map";
            for (auto& trace : search.result.trace)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = trace[0];
                pose.pose.position.y = trace[1];
                pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), trace[2]));
                path.poses.push_back(pose);
                t.poses.push_back(pose.pose);   
            }
            pub_path_->publish(path);
            pub_trace_->publish(t);

            auto goal_msg = uve_message::action::UvePathTrack::Goal();
            goal_msg.plan_result.resize(1);
            goal_msg.plan_result[0].trace.resize(search.result.trace.size());
            for (int i = 0; i < search.result.trace.size(); i++)
            {
                goal_msg.plan_result[0].trace[i].x = search.result.trace[i][0];
                goal_msg.plan_result[0].trace[i].y = search.result.trace[i][1];
                goal_msg.plan_result[0].trace[i].theta = search.result.trace[i][2];
            }
            auto send_goal_options = rclcpp_action::Client<uve_message::action::UvePathTrack>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&HybridAStarTestNode::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.result_callback = std::bind(&HybridAStarTestNode::result_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&HybridAStarTestNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
            
        }
    }

    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<uve_message::action::UvePathTrack>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<uve_message::action::UvePathTrack>::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                break;
        }
    }

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<uve_message::action::UvePathTrack>::SharedPtr,
        const std::shared_ptr<const uve_message::action::UvePathTrack::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Received feedback: %d", feedback->cur_trace);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_trace_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_status_;

    rclcpp_action::Client<uve_message::action::UvePathTrack>::SharedPtr action_client_;

    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::Pose2D start_;
    geometry_msgs::msg::Pose2D goal_;
};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HybridAStarTestNode>());
    rclcpp::shutdown();
    return 0;
}
