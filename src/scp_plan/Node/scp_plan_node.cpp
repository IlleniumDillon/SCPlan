#include "scp_plan_node.hpp"
#include <fstream>

SCPPlanNode::SCPPlanNode()
    : Node("scp_plan_node")
{
    RCLCPP_INFO(get_logger(), "SCP Plan Node Started");

    declare_parameter("scene_width", 12.0);
    declare_parameter("scene_height", 12.0);
    declare_parameter("agent_v", 0.3);
    declare_parameter("agent_w", M_PI / 8);
    declare_parameter("agent_dt", 0.5);
    declare_parameter("solve_resolution", 0.02);
    declare_parameter("distance_map_resolution", 0.05);
    sceneWidth = get_parameter("scene_width").as_double();
    sceneHeight = get_parameter("scene_height").as_double();
    elementMap.maxX = sceneWidth / 2;
    elementMap.minX = -sceneWidth / 2;
    elementMap.maxY = sceneHeight / 2;
    elementMap.minY = -sceneHeight / 2;
    agentV = get_parameter("agent_v").as_double();
    agentW = get_parameter("agent_w").as_double();
    agentDt = get_parameter("agent_dt").as_double();
    RCLCPP_INFO(get_logger(), "Scene size: %f x %f", sceneWidth, sceneHeight);
    RCLCPP_INFO(get_logger(), "Agent v: %f, w: %f, dt: %f", agentV, agentW, agentDt);
    solveResolution = get_parameter("solve_resolution").as_double();
    distanceMapResolution = get_parameter("distance_map_resolution").as_double();
    RCLCPP_INFO(get_logger(), "Solve resolution: %f", solveResolution);
    RCLCPP_INFO(get_logger(), "Distance map resolution: %f", distanceMapResolution);

    Eigen::Matrix4d transform;
    transform << 1, 0, 0, -sceneWidth / 2,
                 0, -1, 0, sceneHeight / 2,
                 0, 0, -1, 0,
                 0, 0, 0, 1;
    elementMap.visualizeCS = CoordSystem(transform);

    try
    {
        modelPath = ament_index_cpp::get_package_share_directory("scp_simulate")
            + "/models/";
    }
    catch(ament_index_cpp::PackageNotFoundError& e)
    {
        RCLCPP_ERROR(get_logger(), "Package not found: %s", e.what());
    }
    RCLCPP_INFO(get_logger(), "Model path: %s", modelPath.c_str());

    loadModelTemplate("agent", agentTemplate);
    loadModelTemplate("good", goodTemplate);
    loadModelTemplate("obstacle", obstacleTemplate);
    loadModelTemplate("wall10", wall10Template);
    loadModelTemplate("wall12", wall12Template);

    modelStateListSubscription = create_subscription<scp_message::msg::ModelStateList>(
        "model_states", 1, std::bind(&SCPPlanNode::modelStateListCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to model_states");
    agentActionListPublisher = create_publisher<scp_message::msg::AgentActionList>("agent_actions", 1);
    RCLCPP_INFO(get_logger(), "Publishing agent_actions");
    occupancyGridPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 1);
    RCLCPP_INFO(get_logger(), "Publishing occupancy_grid");
    agentTargetPoseSubscription = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 1, std::bind(&SCPPlanNode::agentTargetPoseCallback, this, std::placeholders::_1));
    pathPublisher = create_publisher<nav_msgs::msg::Path>("path", 1);
    RCLCPP_INFO(get_logger(), "Publishing path");

    timer = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SCPPlanNode::timerCallback, this));
}

bool SCPPlanNode::loadModelTemplate(std::string modelName, Element &element)
{
    std::string modelFile = modelPath + modelName + ".json";
    std::ifstream file(modelFile);
    if (!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Failed to open file: %s", modelFile.c_str());
        return false;
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(file, root))
    {
        RCLCPP_ERROR(get_logger(), "Failed to parse file: %s", modelFile.c_str());
        file.close();
        return false;
    }
    file.close();

    if (root["isAgent"].isNull() ||
        root["isStatic"].isNull() ||
        root["originShape"].isNull())
    {
        RCLCPP_ERROR(get_logger(), "Invalid file format: %s", modelFile.c_str());
        return false;
    }

    element.isAgent = root["isAgent"].asBool();
    element.isStatic = root["isStatic"].asBool();

    Json::Value originShape = root["originShape"];
    std::vector<CoordD> vertices;
    for (int i = 0; i < originShape.size(); i++)
    {
        CoordD vertex(
            originShape[i][0].asDouble(),
            originShape[i][1].asDouble());
        vertices.push_back(vertex);
    }
    element.originShape = Polygon(vertices);
    element.id = -1;

    if (!root["originAnchors"].isNull())
    {
        Json::Value originAnchors = root["originAnchors"];
        std::vector<CoordD> anchors;
        for (int i = 0; i < originAnchors.size(); i++)
        {
            CoordD anchor(
                originAnchors[i][0].asDouble(),
                originAnchors[i][1].asDouble());
            anchors.push_back(anchor);
        }
        element.originAnchors = anchors;
    }

    return true;
}

void SCPPlanNode::agentTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    CoordD targetPosition(
        msg->pose.position.x,
        msg->pose.position.y);
    double targetYaw = tf2::getYaw(msg->pose.orientation);
    RCLCPP_INFO(get_logger(), "Target position: %f, %f, yaw: %f", targetPosition.x, targetPosition.y, targetYaw);

    elementMap.agent = agentShadow;
    elementMap.elements = elementsShadow;

    TaskLowLevel taskLL;
    taskLL.start = agentShadow.panning;
    taskLL.goal = targetPosition;
    taskLL.theta0 = agentShadow.rotation;
    taskLL.theta1 = targetYaw;
    taskLL.V = agentV;
    taskLL.W = agentW;
    taskLL.dt = agentDt;

    HybridAStar hybridAStar(elementMap, solveResolution, distanceMapResolution);
    PlanResult planResult = hybridAStar.plan(taskLL);

    RCLCPP_INFO(get_logger(), "Plan result: %s", planResult.success ? "success" : "failed");

    if (planResult.success)
    {
        path.header.stamp = now();
        path.header.frame_id = "map";
        path.poses.clear();
        agentActionList.actions.clear();
        for (int i = 1; i < planResult.actions.size(); i++)
        {
            scp_message::msg::AgentAction agentAction;
            agentAction.agent_name = "agent_0";
            agentAction.action = 0;
            agentAction.v = planResult.actions[i].V;
            agentAction.w = planResult.actions[i].W;
            agentAction.dt = agentDt;

            geometry_msgs::msg::Pose cameFrom;
            cameFrom.position.x = planResult.actions[i].from.x;
            cameFrom.position.y = planResult.actions[i].from.y;
            cameFrom.position.z = 0;
            tf2::Quaternion q0;
            q0.setRPY(0.0, 0.0, planResult.actions[i].theta0);
            cameFrom.orientation = tf2::toMsg(q0);
            agentAction.came_from = cameFrom;

            geometry_msgs::msg::Pose goingTo;
            goingTo.position.x = planResult.actions[i].to.x;
            goingTo.position.y = planResult.actions[i].to.y;
            goingTo.position.z = 0;
            tf2::Quaternion q1;
            q1.setRPY(0.0, 0.0, planResult.actions[i].theta1);
            goingTo.orientation = tf2::toMsg(q1);
            agentAction.going_to = goingTo;

            agentAction.final_flag = planResult.actions[i]._final;

            agentActionList.actions.push_back(agentAction);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = now();
            pose.header.frame_id = "map";
            pose.pose = goingTo;
            path.poses.push_back(pose);
        }
        // drawDynamic(elementMap, planResult);
        agentActionListPublisher->publish(agentActionList);
        pathPublisher->publish(path);
    }
}

void SCPPlanNode::modelStateListCallback(const scp_message::msg::ModelStateList::SharedPtr msg)
{
    modelStateList = *msg;
    elementsShadow.clear();
    for (int i = 0; i < modelStateList.modelstates.size(); i++)
    {
        if (modelStateList.modelstates[i].name == "agent_0")
        {
            agentShadow.id = i;
            agentShadow.originShape = agentTemplate.originShape;
            agentShadow.originAnchors = agentTemplate.originAnchors;
            CoordD panning(
                modelStateList.modelstates[i].pose.position.x,
                modelStateList.modelstates[i].pose.position.y);
            double rotation = tf2::getYaw(modelStateList.modelstates[i].pose.orientation);
            agentShadow.setGeometry(panning, rotation);
        }
        else
        {
            Element element;
            element.id = i;
            std::string type = modelStateList.modelstates[i].name.substr(0, modelStateList.modelstates[i].name.find("_"));
            if (type == "good")
            {
                element.name = modelStateList.modelstates[i].name;
                element.originShape = goodTemplate.originShape;
                element.originAnchors = goodTemplate.originAnchors;
            }
            else if (type == "obstacle")
            {
                element.name = "";
                element.originShape = obstacleTemplate.originShape;
                element.originAnchors = obstacleTemplate.originAnchors;
            }
            else if (type == "wall10")
            {
                element.name = "";
                element.originShape = wall10Template.originShape;
                element.originAnchors = wall10Template.originAnchors;
            }
            else if (type == "wall12")
            {
                element.name = "";
                element.originShape = wall12Template.originShape;
                element.originAnchors = wall12Template.originAnchors;
            }
            else
            {
                continue;
            }
            CoordD panning(
                modelStateList.modelstates[i].pose.position.x,
                modelStateList.modelstates[i].pose.position.y);
            double rotation = tf2::getYaw(modelStateList.modelstates[i].pose.orientation);
            element.setGeometry(panning, rotation);
            elementsShadow.push_back(element);
        }
    }
}

void SCPPlanNode::timerCallback()
{
    //RCLCPP_INFO(get_logger(), "Element number: %d", elementsShadow.size());
    occupancyGrid.header.stamp = now();
    occupancyGrid.header.frame_id = "map";
    occupancyGrid.info.height = sceneHeight / distanceMapResolution;
    occupancyGrid.info.width = sceneWidth / distanceMapResolution;
    occupancyGrid.info.resolution = distanceMapResolution;
    occupancyGrid.info.origin.position.x = -sceneWidth / 2;
    occupancyGrid.info.origin.position.y = -sceneHeight / 2;
    occupancyGrid.info.origin.position.z = 0;
    occupancyGrid.info.origin.orientation.x = 0;
    occupancyGrid.info.origin.orientation.y = 0;
    occupancyGrid.info.origin.orientation.z = 0;
    occupancyGrid.info.origin.orientation.w = 1;
    occupancyGrid.data.clear();
    occupancyGrid.data.resize(occupancyGrid.info.height * occupancyGrid.info.width, 0);

    std::vector<Element> elements = elementsShadow;
    elements.push_back(agentShadow);

    std::vector<CoordD> edge;
    for (int i = 0; i < elements.size(); i++)
    {
        for (int j = 0; j < elements[i].shape.vertices.size(); j++)
        {
            Line line(
                elements[i].shape.vertices[j],
                elements[i].shape.vertices[(j + 1) % elements[i].shape.vertices.size()]);
            std::vector<Coord<double>> discretized = line.discretize(distanceMapResolution);
            edge.insert(edge.end(), discretized.begin(), discretized.end());
        }
    }
    for (int i = 0; i < edge.size(); i++)
    {
        CoordI index(
            (edge[i].x - occupancyGrid.info.origin.position.x) / distanceMapResolution,
            (edge[i].y - occupancyGrid.info.origin.position.y) / distanceMapResolution
        );
        if (index.x >= 0 && index.x < occupancyGrid.info.width &&
            index.y >= 0 && index.y < occupancyGrid.info.height)
        {
            occupancyGrid.data[index.y * occupancyGrid.info.width + index.x] = 100;
        }
    }
    occupancyGridPublisher->publish(occupancyGrid);
}
