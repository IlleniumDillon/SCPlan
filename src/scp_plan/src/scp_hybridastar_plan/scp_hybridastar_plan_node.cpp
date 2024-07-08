#include "scp_hybridastar_plan_node.hpp"
#include <fstream>

SCPHAPlanNode::SCPHAPlanNode()
    : Node("scp_plan_node")
{
    loadConfig();
    sub_model_state_list_ = this->create_subscription<scp_message::msg::ModelStateList>
        ("model_states", 1, std::bind(&SCPHAPlanNode::modelStateListCallback, this, std::placeholders::_1));
    pub_grid_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 1);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SCPHAPlanNode::timerCallback, this));

    grid_map_ = GridMap(-scene_width_ / 2, -scene_height_ / 2, scene_width_ / 2, scene_height_ / 2, position_resolution_, yaw_step_);
    hybrid_astar_.config(agent_v, agent_w, agent_dt, 2, 3);

    sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
        ("goal_pose", 1, std::bind(&SCPHAPlanNode::goalCallback, this, std::placeholders::_1));
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);
}

void SCPHAPlanNode::loadConfig()
{
    declare_parameter("scene_width", 12.0);
    declare_parameter("scene_height", 12.0);
    declare_parameter("position_resolution", 0.05);
    declare_parameter("yaw_step", 36);
    declare_parameter("agent_v", 0.5);
    declare_parameter("agent_w", M_PI / 8);
    declare_parameter("agent_dt", 0.5);

    scene_width_ = get_parameter("scene_width").as_double();
    scene_height_ = get_parameter("scene_height").as_double();
    position_resolution_ = get_parameter("position_resolution").as_double();
    yaw_step_ = get_parameter("yaw_step").as_int();
    agent_v = get_parameter("agent_v").as_double();
    agent_w = get_parameter("agent_w").as_double();
    agent_dt = get_parameter("agent_dt").as_double();

    try
    {
        config_file_path_ = ament_index_cpp::get_package_share_directory("scp_simulate")
            + "/models/";
    }
    catch(ament_index_cpp::PackageNotFoundError& e)
    {
        RCLCPP_ERROR(get_logger(), "Package not found: %s", e.what());
    }
    RCLCPP_INFO(get_logger(), "Model path: %s", config_file_path_.c_str());

    loadModelConfig("agent", &agent_shape_, &agent_anchor_);
    loadModelConfig("good", &good_shape_, &good_anchor_);
    loadModelConfig("wall10", &wall10_shape_, nullptr);
    loadModelConfig("wall12", &wall12_shape_, nullptr);
    loadModelConfig("obstacle", &obstacle_shape_, nullptr);
}

void SCPHAPlanNode::loadModelConfig(std::string model_name, std::vector<Point> *shape, std::vector<Point> *anchor)
{
    std::string modelFile = config_file_path_ + model_name + ".json";
    std::ifstream file(modelFile);
    if (!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Failed to open file: %s", modelFile.c_str());
        return;
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(file, root))
    {
        RCLCPP_ERROR(get_logger(), "Failed to parse file: %s", modelFile.c_str());
        file.close();
        return;
    }
    file.close();

    if (root["isAgent"].isNull() ||
        root["isStatic"].isNull() ||
        root["originShape"].isNull())
    {
        RCLCPP_ERROR(get_logger(), "Invalid file format: %s", modelFile.c_str());
        return;
    }

    Json::Value originShape = root["originShape"];
    (*shape).clear();
    for (int i = 0; i < originShape.size(); i++)
    {
        Point point;
        point.x = originShape[i][0].asDouble();
        point.y = originShape[i][1].asDouble();
        (*shape).push_back(point);
    }

    if (!root["originAnchors"].isNull() && anchor != nullptr)
    {
        Json::Value originAnchors = root["originAnchors"];
        (*anchor).clear();
        for (int i = 0; i < originAnchors.size(); i++)
        {
            Point point;
            point.x = originAnchors[i][0].asDouble();
            point.y = originAnchors[i][1].asDouble();
            (*anchor).push_back(point);
        }
    }
}

void SCPHAPlanNode::modelStateListCallback(const scp_message::msg::ModelStateList::SharedPtr msg)
{
    model_state_list_msg_ = *msg;
    static int enterCount = 0;

    if (enterCount <= 2)
    {
        static_elements_.clear();
        dynamic_elements_.clear();
        for (auto& model_state : model_state_list_msg_.static_modelstates)
        {
            Element element;
            element.id = model_state.id;
            std::string type = model_state.name.substr(0, model_state.name.find_first_of("_"));
            if (type == "wall10")
            {
                element.originVertices = wall10_shape_;
            }
            else if (type == "wall12")
            {
                element.originVertices = wall12_shape_;
            }
            else if (type == "obstacle")
            {
                element.originVertices = obstacle_shape_;
            }
            else
            {
                continue;
            }
            Pose2D pose;
            pose.x = model_state.pose.position.x;
            pose.y = model_state.pose.position.y;
            pose.theta = tf2::getYaw(model_state.pose.orientation);
            element.updatePose(pose);
            static_elements_.push_back(element);
            grid_map_.putElement(element);
        }
        for (auto& model_state : model_state_list_msg_.dynamic_modelstates)
        {
            Element element;
            element.id = model_state.id;
            std::string type = model_state.name.substr(0, model_state.name.find_first_of("_"));
            if (type == "good")
            {
                element.originVertices = good_shape_;
            }
            else
            {
                continue;
            }
            Pose2D pose;
            pose.x = model_state.pose.position.x;
            pose.y = model_state.pose.position.y;
            pose.theta = tf2::getYaw(model_state.pose.orientation);
            element.updatePose(pose);
            dynamic_elements_.push_back(element);
            grid_map_.putElement(element);  
        }
        if (model_state_list_msg_.agent_modelstate.id != -1)
        {
            agent_.id = model_state_list_msg_.agent_modelstate.id;
            agent_.originVertices = agent_shape_;
            Pose2D pose;
            pose.x = model_state_list_msg_.agent_modelstate.pose.position.x;
            pose.y = model_state_list_msg_.agent_modelstate.pose.position.y;
            pose.theta = tf2::getYaw(model_state_list_msg_.agent_modelstate.pose.orientation);
            agent_.updatePose(pose);
        }
        if (enterCount == 2)
        {
            RCLCPP_INFO(this->get_logger(), "Elements init done.");
        }
        enterCount++;
        return;
    }

    for (auto& model_state : model_state_list_msg_.dynamic_modelstates)
    {
        Pose2D pose;
        pose.x = model_state.pose.position.x;
        pose.y = model_state.pose.position.y;
        pose.theta = tf2::getYaw(model_state.pose.orientation);
        for (auto& element : dynamic_elements_)
        {
            if (element.id == model_state.id)
            {
                element.updatePose(pose);
                grid_map_.putElement(element);
                break;
            }
        }
    }
    if (model_state_list_msg_.agent_modelstate.id != -1)
    {
        Pose2D pose;
        pose.x = model_state_list_msg_.agent_modelstate.pose.position.x;
        pose.y = model_state_list_msg_.agent_modelstate.pose.position.y;
        pose.theta = tf2::getYaw(model_state_list_msg_.agent_modelstate.pose.orientation);
        agent_.updatePose(pose);
    }
}

void SCPHAPlanNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Goal received.");
    goal_msg_ = *msg;
    Pose2D goal;
    goal.x = goal_msg_.pose.position.x;
    goal.y = goal_msg_.pose.position.y;
    goal.theta = tf2::getYaw(goal_msg_.pose.orientation);
    hybrid_astar_.plan(grid_map_, agent_.pose, goal, agent_);
    if (hybrid_astar_.plan_result.success)
    {
        RCLCPP_INFO(this->get_logger(), "Plan success. iterations: %d", hybrid_astar_.plan_result.iterations);
        hybrid_astar_.toMsg(path_msg_);
        pub_path_->publish(path_msg_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Plan failed. iterations: %d", hybrid_astar_.plan_result.iterations);
    }
}

void SCPHAPlanNode::timerCallback()
{
    grid_map_.toMsg(grid_map_msg_);
    pub_grid_map_->publish(grid_map_msg_);
}
