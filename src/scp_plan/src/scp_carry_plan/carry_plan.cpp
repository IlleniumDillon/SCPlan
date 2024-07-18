#include "carry_plan.hpp"
#include <chrono>

using namespace scp;

scp::CarryPlan::CarryPlan()
{
}

void CarryPlan::config(double v, double w, double dt)
{
    this->v = v;
    this->w = w;
    this->dt = dt;
}

void CarryPlan::updateElement(std::vector<Element> &dynamic_elements, std::vector<Element> &static_elements, Element &agent)
{
    this->dynamic_elements = dynamic_elements;
    this->static_elements = static_elements;
    this->agent = agent;
    this->obstacles.clear();
    this->obstacles.insert(this->obstacles.end(), this->static_elements.begin(), this->static_elements.end());
    this->obstacles.insert(this->obstacles.end(), this->dynamic_elements.begin(), this->dynamic_elements.end());
}

void CarryPlan::plan(GridMap &grid_map, scp_message::msg::ScpCarryTask &task)
{
    auto start = std::chrono::steady_clock::now();

    this->grid_map = grid_map;
    this->task = task;

    plan_result = CarryPlanResult();

    /// step 1: generate sub-goal points according to the anchors of the object
    //// step 1.1: find the target object
    Element targetObject;
    Element targetObjectGoal;
    targetObject.id = -1;
    for (auto &element : this->dynamic_elements)
    {
        if (element.id == task.who)
        {
            targetObject = element;
            break;
        }
    }
    if (targetObject.id == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("carry_plan"), "Object not found.");
        return;
    }
    // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Target pose: (%f, %f, %f).", targetObject.pose.x, targetObject.pose.y, targetObject.pose.theta);
    targetObjectGoal = targetObject;
    Pose2D targetObjectPose;
    targetObjectPose.x = task.going_to.position.x;
    targetObjectPose.y = task.going_to.position.y;
    targetObjectPose.theta = tf2::getYaw(task.going_to.orientation);
    targetObjectGoal.updatePose(targetObjectPose);
    // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Target goal pose: (%f, %f, %f).", targetObjectGoal.pose.x, targetObjectGoal.pose.y, targetObjectGoal.pose.theta);
    //// step 1.2: generate sub-goal points
    std::vector<Pose2D> subGoalPoses;
    std::vector<Pose2D> GoalPoses;

    double agentCenterAgentDistance = sqrt(pow(this->agent.originAnchors[0].x, 2) + pow(this->agent.originAnchors[0].y, 2)) + 0.05;
    // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Agent center to agent distance: %f.", agentCenterAgentDistance);

    for (int i = 0; i < targetObject.originAnchors.size(); i++)
    {
        double targetCenterAnchorTheta = atan2(targetObject.originAnchors[i].y, targetObject.originAnchors[i].x) + targetObject.pose.theta;
        if (targetCenterAnchorTheta > M_PI)
        {
            targetCenterAnchorTheta -= 2 * M_PI;
        }
        else if (targetCenterAnchorTheta < -M_PI)
        {
            targetCenterAnchorTheta += 2 * M_PI;
        }
        Pose2D subGoalPose;
        subGoalPose.x = targetObject.currentAnchors[i].x + agentCenterAgentDistance * cos(targetCenterAnchorTheta);
        subGoalPose.y = targetObject.currentAnchors[i].y + agentCenterAgentDistance * sin(targetCenterAnchorTheta);
        subGoalPose.theta = targetCenterAnchorTheta + M_PI;
        if (subGoalPose.theta > M_PI)
        {
            subGoalPose.theta -= 2 * M_PI;
        }
        else if (subGoalPose.theta < -M_PI)
        {
            subGoalPose.theta += 2 * M_PI;
        }
        subGoalPoses.push_back(subGoalPose);
        // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Sub goal pose: (%f, %f, %f).", subGoalPose.x, subGoalPose.y, subGoalPose.theta);

        targetCenterAnchorTheta = atan2(targetObjectGoal.originAnchors[i].y, targetObjectGoal.originAnchors[i].x) + targetObjectGoal.pose.theta;
        if (targetCenterAnchorTheta > M_PI)
        {
            targetCenterAnchorTheta -= 2 * M_PI;
        }
        else if (targetCenterAnchorTheta < -M_PI)
        {
            targetCenterAnchorTheta += 2 * M_PI;
        }
        Pose2D GoalPose;
        GoalPose.x = targetObjectGoal.currentAnchors[i].x + agentCenterAgentDistance * cos(targetCenterAnchorTheta);
        GoalPose.y = targetObjectGoal.currentAnchors[i].y + agentCenterAgentDistance * sin(targetCenterAnchorTheta);
        GoalPose.theta = targetCenterAnchorTheta + M_PI;
        if (GoalPose.theta > M_PI)
        {
            GoalPose.theta -= 2 * M_PI;
        }
        else if (GoalPose.theta < -M_PI)
        {
            GoalPose.theta += 2 * M_PI;
        }
        GoalPoses.push_back(GoalPose);
        // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Goal pose: (%f, %f, %f).", GoalPose.x, GoalPose.y, GoalPose.theta);
    }
    
    /// step 2: start planning threads to generate path
    int currentBatch = 0;
    int conditionLeft = subGoalPoses.size();
    std::vector<std::future<void>> planThreads(MAX_THREAD_NUM);
    std::vector<CarryPlanResult> planResults(GoalPoses.size());

    while (conditionLeft > 0)
    {
        for (int i = 0; i < MAX_THREAD_NUM; i++)
        {
            if (conditionLeft <= 0)
            {
                break;
            }
            // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Start thread %d.", currentBatch);
            
            planThreads[i] = std::async(
                std::launch::async, 
                &CarryPlan::planThread, 
                this, 
                std::ref(grid_map), 
                std::ref(agent.pose), 
                std::ref(subGoalPoses[currentBatch]), 
                std::ref(GoalPoses[currentBatch]), 
                std::ref(planResults[currentBatch]));
            // planThread(grid_map, agent.pose, subGoalPoses[currentBatch], GoalPoses[currentBatch], planResults[currentBatch]);
            currentBatch++;
            conditionLeft--;
        }
        for (int i = 0; i < MAX_THREAD_NUM; i++)
        {
            planThreads[i].wait();
        }
    }
    /// step 3: select the best path
    double minCost = std::numeric_limits<double>::max();
    int minCostIndex = -1;
    for (int i = 0; i < GoalPoses.size(); i++)
    {
        if (planResults[i].success == false)
        {
            continue;
        }
        if (planResults[i].cost < minCost)
        {
            minCost = planResults[i].cost;
            minCostIndex = i;
        }
    }
    if (minCostIndex == -1)
    {
        // RCLCPP_ERROR(rclcpp::get_logger("carry_plan"), "No path found.");
        return;
    }
    plan_result = planResults[minCostIndex];
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    plan_result.planTime = duration.count() / 1000000.0;
    RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Plan time: %f s.", plan_result.planTime);
}

void CarryPlan::toMsg(scp_message::msg::AgentActionList &action_list_msg)
{
}

void scp::CarryPlan::toMsg(nav_msgs::msg::Path &path_msg)
{
    path_msg.header.stamp = rclcpp::Clock().now();
    path_msg.header.frame_id = "map";
    path_msg.poses.clear();
    for (auto pose : plan_result.path)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.pose.position.x = pose.second.x;
        pose_msg.pose.position.y = pose.second.y;
        pose_msg.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.second.theta);
        pose_msg.pose.orientation = tf2::toMsg(q);
        path_msg.poses.push_back(pose_msg);
        // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Path pose: (%f, %f, %f).", pose.second.x, pose.second.y, pose.second.theta);
    }
}

void scp::CarryPlan::planThread(GridMap &grid_map, Pose2D &start, Pose2D &sub, Pose2D &goal, CarryPlanResult &rst)
{
    rst = CarryPlanResult();

    // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Start pose: (%f, %f, %f). Sub goal pose: (%f, %f, %f). Goal pose: (%f, %f, %f).", start.x, start.y, start.theta, sub.x, sub.y, sub.theta, goal.x, goal.y, goal.theta);
    RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "[%d]Thread start.", std::this_thread::get_id());

    Element agentBackup = agent;
    agentBackup.updatePose(sub);
    if (checkCollision(agentBackup))
    {
        RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "[%d]Sub goal collision.", std::this_thread::get_id());
        rst = CarryPlanResult();
        return;
    }
    Element newAgent = unionElement(agentBackup, obstacles[task.who]);
    // for (int i = 0; i < newAgent.currentVertices.size(); i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Agent anchor: (%f, %f).", newAgent.currentVertices[i].x, newAgent.currentVertices[i].y);
    // }
    Element newAgentBackup = newAgent;
    newAgentBackup.updatePose(goal);
    if (checkCollision(newAgentBackup))
    {
        RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "[%d]Goal collision.", std::this_thread::get_id());
        rst = CarryPlanResult();
        return;
    }

    HybridAStar hybrid_astar;
    hybrid_astar.config(v, w, dt, 2, 3);
    hybrid_astar.updateElement(obstacles, agent);
    hybrid_astar.plan(grid_map, start, sub, agent);
    if (hybrid_astar.plan_result.success == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "[%d]Sub goal failed.", std::this_thread::get_id());
        rst = CarryPlanResult();
        return;
    }
    else
    {
        rst.cost += hybrid_astar.plan_result.cost;
        rst.iterations += hybrid_astar.plan_result.iterations;

        for (auto &pose : hybrid_astar.plan_result.path)
        {
            rst.path.push_back(std::make_pair(false, pose));
        }
    }

    hybrid_astar.grid_map.removeElement(obstacles[task.who]);
    hybrid_astar.updateElement(obstacles, newAgent);
    hybrid_astar.plan(sub, goal);
    if (hybrid_astar.plan_result.success == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "[%d]Goal failed.", std::this_thread::get_id());
        rst = CarryPlanResult();
        return;
    }
    else
    {
        rst.cost += hybrid_astar.plan_result.cost;
        rst.iterations += hybrid_astar.plan_result.iterations;

        for (auto &pose : hybrid_astar.plan_result.path)
        {
            rst.path.push_back(std::make_pair(false, pose));
        }

        rst.success = true;
    }

    RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "[%d]Thread end.", std::this_thread::get_id());
}

bool CarryPlan::checkCollision(Element& e)
{
    for (auto point : e.currentVertices)
    {
        auto index = grid_map(point);
        if (index.node == nullptr)
        {
            return true;
        }
        if (index.occupied)
        {
            return true;
        }
    }

    return false;
}
