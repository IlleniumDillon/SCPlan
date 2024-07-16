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
    targetObjectGoal = targetObject;
    Pose2D targetObjectPose;
    targetObjectPose.x = task.going_to.position.x;
    targetObjectPose.y = task.going_to.position.y;
    targetObjectPose.theta = tf2::getYaw(task.going_to.orientation);
    targetObjectGoal.updatePose(targetObjectPose);
    //// step 1.2: generate sub-goal points
    std::vector<Pose2D> subGoalPoses;
    std::vector<Pose2D> GoalPoses;
    for (auto &anchor : targetObject.originAnchors)
    {
        Pose2D subGoalPose;
        
        Point targetObjectCenter;
        targetObjectCenter.x = targetObject.pose.x;
        targetObjectCenter.y = targetObject.pose.y;
        Point anchorPoint;
        anchorPoint.x = anchor.x;
        anchorPoint.y = anchor.y;

        double agentAnchorDistance = std::sqrt(std::pow(agent.originAnchors[0].x, 2) + std::pow(agent.originAnchors[0].y, 2));
        double objectAnchorAngle = std::atan2(anchor.y - targetObjectCenter.y, anchor.x - targetObjectCenter.x);
        double agentAnchorAngle = std::atan2(targetObjectCenter.y - anchor.y, targetObjectCenter.x - anchor.x);

        subGoalPose.x = anchorPoint.x + agentAnchorDistance * std::cos(objectAnchorAngle);
        subGoalPose.y = anchorPoint.y + agentAnchorDistance * std::sin(objectAnchorAngle);
        subGoalPose.theta = agentAnchorAngle;

        subGoalPoses.push_back(subGoalPose);
    }
    for (auto &anchor : targetObjectGoal.originAnchors)
    {
        Pose2D GoalPose;
        
        Point targetObjectCenter;
        targetObjectCenter.x = targetObjectGoal.pose.x;
        targetObjectCenter.y = targetObjectGoal.pose.y;
        Point anchorPoint;
        anchorPoint.x = anchor.x;
        anchorPoint.y = anchor.y;

        double agentAnchorDistance = std::sqrt(std::pow(agent.originAnchors[0].x, 2) + std::pow(agent.originAnchors[0].y, 2));
        double objectAnchorAngle = std::atan2(anchor.y - targetObjectCenter.y, anchor.x - targetObjectCenter.x);
        double agentAnchorAngle = std::atan2(targetObjectCenter.y - anchor.y, targetObjectCenter.x - anchor.x);

        GoalPose.x = anchorPoint.x + agentAnchorDistance * std::cos(objectAnchorAngle);
        GoalPose.y = anchorPoint.y + agentAnchorDistance * std::sin(objectAnchorAngle);
        GoalPose.theta = agentAnchorAngle;

        GoalPoses.push_back(GoalPose);
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
            Pose2D subGoalPose = subGoalPoses[currentBatch];
            Pose2D GoalPose = GoalPoses[currentBatch];
            planThreads[i] = std::async(
                std::launch::async, 
                &CarryPlan::planThread, 
                this, 
                std::ref(grid_map), 
                std::ref(agent.pose), 
                std::ref(subGoalPose), 
                std::ref(GoalPose), 
                std::ref(planResults[currentBatch]));
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
}
