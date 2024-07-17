#include "carry_plan.hpp"

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
    RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Target pose: (%f, %f, %f).", targetObject.pose.x, targetObject.pose.y, targetObject.pose.theta);
    targetObjectGoal = targetObject;
    Pose2D targetObjectPose;
    targetObjectPose.x = task.going_to.position.x;
    targetObjectPose.y = task.going_to.position.y;
    targetObjectPose.theta = tf2::getYaw(task.going_to.orientation);
    targetObjectGoal.updatePose(targetObjectPose);
    RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Target goal pose: (%f, %f, %f).", targetObjectGoal.pose.x, targetObjectGoal.pose.y, targetObjectGoal.pose.theta);
    //// step 1.2: generate sub-goal points
    std::vector<Pose2D> subGoalPoses;
    std::vector<Pose2D> GoalPoses;

    double agentCenterAgentDistance = sqrt((this->agent.originAnchors[0].x, 2) + pow(this->agent.originAnchors[0].y, 2)) + 0.05;

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
        RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Sub goal pose: (%f, %f, %f).", subGoalPose.x, subGoalPose.y, subGoalPose.theta);

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
        RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Goal pose: (%f, %f, %f).", GoalPose.x, GoalPose.y, GoalPose.theta);
    }
    
    /// step 2: start planning threads to generate path
    int currentBatch = 0;
    int conditionLeft = subGoalPoses.size();
    // std::vector<std::future<void>> planThreads(MAX_THREAD_NUM);
    std::vector<CarryPlanResult> planResults(GoalPoses.size());

    while (conditionLeft > 0)
    {
        for (int i = 0; i < MAX_THREAD_NUM; i++)
        {
            if (conditionLeft <= 0)
            {
                break;
            }
            RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Start thread %d.", currentBatch);
            Pose2D subGoalPose = subGoalPoses[currentBatch];
            Pose2D GoalPose = GoalPoses[currentBatch];
            // planThreads[i] = std::async(
            //     std::launch::async, 
            //     &CarryPlan::planThread, 
            //     this, 
            //     std::ref(grid_map), 
            //     std::ref(agent.pose), 
            //     std::ref(subGoalPose), 
            //     std::ref(GoalPose), 
            //     std::ref(planResults[currentBatch]));
            planThread(grid_map, agent.pose, subGoalPose, GoalPose, planResults[currentBatch]);
            currentBatch++;
            conditionLeft--;
        }
        // for (int i = 0; i < MAX_THREAD_NUM; i++)
        // {
        //     planThreads[i].wait();
        // }
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
        RCLCPP_ERROR(rclcpp::get_logger("carry_plan"), "No path found.");
        return;
    }
    plan_result = planResults[minCostIndex];
}

void CarryPlan::toMsg(scp_message::msg::AgentActionList &action_list_msg)
{
}

void scp::CarryPlan::planThread(GridMap &grid_map, Pose2D &start, Pose2D &sub, Pose2D &goal, CarryPlanResult &rst)
{
    RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Start pose: (%f, %f, %f). Sub goal pose: (%f, %f, %f). Goal pose: (%f, %f, %f).", start.x, start.y, start.theta, sub.x, sub.y, sub.theta, goal.x, goal.y, goal.theta);
    RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "[%d]Thread start.", std::this_thread::get_id());

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

    Element agentBackup = agent;
    agentBackup.updatePose(sub);
    Element newAgent = unionElement(agentBackup, obstacles[task.who]);
    for (int i = 0; i < newAgent.currentVertices.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Agent anchor: (%f, %f).", newAgent.currentVertices[i].x, newAgent.currentVertices[i].y);
    }

    //hybrid_astar.grid_map.removeElement(obstacles[task.who]);

    RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "[%d]Thread end.", std::this_thread::get_id());
}
