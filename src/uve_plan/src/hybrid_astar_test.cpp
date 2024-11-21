#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "hybrid_astar/hybrid_astar.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp" 
#include "nav_msgs/msg/path.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/pose_array.hpp"
// #include "uve_message/msg/uve_agent_status.hpp"
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
        if (goal_.theta >= 2*M_PI)
        {
            goal_.theta -= 2 * M_PI;
        }
        if (start_.theta < 0)
        {
            start_.theta += 2 * M_PI;
        }
        if (start_.theta >= 2*M_PI)
        {
            start_.theta -= 2 * M_PI;
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
        start->state = {start_.x, start_.y, start_.theta};
        if (start == nullptr)
        {
            std::cout << "start is null" << std::endl;
            return;
        }
        std::cout << "start: " << start->state << std::endl;
        auto goal = graph({(int)((goal_.x - origin[0]) / resolution[0]), (int)((goal_.y - origin[1]) / resolution[1]), (int)(goal_.theta / resolution[2])});
        goal->state = {goal_.x, goal_.y, goal_.theta};
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
            
            double ddt = 0.1;
            geometry_msgs::msg::PoseArray trace_patch;
            nav_msgs::msg::Path path;
            trace_patch.header.frame_id = "map";
            path.header.frame_id = "map";
            search.result.vw.push_back(Eigen::Matrix<double, 1, 2>(0, 0));
            for (int i = 0; i < search.result.trace.size(); i++)
            {
                Eigen::Vector3i temp;
                temp << (int)((search.result.trace[i](0) - origin[0]) / resolution[0]), (int)((search.result.trace[i](1) - origin[1]) / resolution[1]), (int)(search.result.trace[i](2) / resolution[2]);
                // RCLCPP_INFO(get_logger(), "trace: %f,%f,%f", search.result.trace[i](0), search.result.trace[i](1), search.result.trace[i](2));
                // RCLCPP_INFO(get_logger(), "temp: %f,%f,%f", temp(0)*resolution[0]+origin[0], temp(1)*resolution[1]+origin[1], temp(2)*resolution[2]);
                double v = search.result.vw[i+1](0);
                double w = search.result.vw[i+1](1);
                // RCLCPP_INFO(get_logger(), "v: %f, w: %f", v, w);
                // RCLCPP_INFO(get_logger(), "------------------");
                
                for (int j = 0; j < 6; j++)
                {
                    double dt = ddt * j;
                    Eigen::Matrix<double, 1, 3> neighbor;
                    neighbor(2) = w * dt;
                    neighbor(1) = neighbor(2) / 2;
                    if (w == 0)
                    {
                        neighbor(0) = v * dt;
                    }
                    else
                    {
                        double R = v / w;
                        double L = std::sqrt(2*R*R*(1 - std::cos(w * dt)));
                        neighbor(0) = L;
                    }
                    Eigen::Matrix<double, 1, 3> dstate;
                    dstate << neighbor(0) * std::cos(search.result.trace[i](2) + neighbor(1)), neighbor(0) * std::sin(search.result.trace[i](2) + neighbor(1)), neighbor(2);

                    Eigen::Matrix<double, 1, 3> newState = search.result.trace[i] + dstate;
                    // RCLCPP_INFO(get_logger(), "newState: %f,%f,%f", newState(0), newState(1), newState(2));
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.pose.position.x = newState(0);
                    pose.pose.position.y = newState(1);
                    pose.pose.position.z = 0;
                    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), newState(2)));
                    path.poses.push_back(pose);
                    trace_patch.poses.push_back(pose.pose);
                }
            }
            pub_path_->publish(path);
            pub_trace_->publish(trace_patch);

            auto goal_msg = uve_message::action::UvePathTrack::Goal();
            goal_msg.plan_result.resize(1);
            goal_msg.plan_result[0].trace.resize(path.poses.size());
            for (int i = 0; i < path.poses.size(); i++)
            {
                goal_msg.plan_result[0].trace[i].x = path.poses[i].pose.position.x;
                goal_msg.plan_result[0].trace[i].y = path.poses[i].pose.position.y;
                goal_msg.plan_result[0].trace[i].theta = tf2::getYaw(path.poses[i].pose.orientation);
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
