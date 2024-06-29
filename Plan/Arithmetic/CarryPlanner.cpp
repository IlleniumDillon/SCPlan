#include "CarryPlanner.hpp"
#include <thread>
#include <future>

CarryPlanner::CarryPlanner(ElementMap &map, double solveResolution, double distanceMapResolution)
{
    this->map = map;
    // distanceMap = DistanceElementMap(map, distanceMapResolution); 
    // gridMap = GridMap(map, solveResolution);
    std::future<void> distanceMapFuture = std::async(
        std::launch::async,
        genreateDistanceElementMap,
        std::ref(map),
        distanceMapResolution,
        std::ref(distanceMap)
    );
    std::future<void> gridMapFuture = std::async(
        std::launch::async,
        generateGridMap,
        std::ref(map),
        solveResolution,
        std::ref(gridMap)
    );
    distanceMapFuture.wait();
    gridMapFuture.wait();
}

PlanResult CarryPlanner::plan(TaskCarry &task)
{
    auto targetObject = *std::find_if(
        map.elements.begin(),
        map.elements.end(),
        [task](Element &element) { return element.id == task.elementId; }
    );
    int numAnchor = targetObject.anchors.size();
    if (numAnchor == 0)
    {
        return PlanResult();
    }

    std::vector<PlanThreadInput> inputs(numAnchor);
    std::vector<PlanResult> results(numAnchor);
    /// fill inputs
    std::vector<CoordD> beforeCarryPositions = generateTargetPositions(targetObject, targetObject.panning, targetObject.rotation);
    std::vector<CoordD> afterCarryPositions = generateTargetPositions(targetObject, task.elementTargetPosition, task.elementTargetTheta);
    for (int i = 0; i < numAnchor; i++)
    {
        PlanThreadInput& input = inputs[i];
        input.elementId = task.elementId;
        input.V = task.V;
        input.W = task.W;
        input.dt = task.dt;
        input.agentPosition0 = task.agentStart;
        input.agentTheta0 = task.agentTheta0;
        input.agentPosition1 = CoordD(beforeCarryPositions[i].x, beforeCarryPositions[i].y);
        input.agentTheta1 = beforeCarryPositions[i].z;
        input.agentPosition2 = CoordD(afterCarryPositions[i].x, afterCarryPositions[i].y);
        input.agentTheta2 = afterCarryPositions[i].z;
        input.agentPosition3 = task.agentGoal;
        input.agentTheta3 = task.agentTheta1;
    }

    int left = numAnchor;
    int current = 0;
    while (left >= MAX_THREAD_NUM)
    {
        std::vector<std::future<PlanResult>> futures(MAX_THREAD_NUM);
        for (int i = 0; i < MAX_THREAD_NUM; i++)
        {
            futures[i] = std::async(
                std::launch::async,
                std::bind(&CarryPlanner::planThread, this, std::ref(inputs[current]))
            );
            current++;
        }
        for (int i = 0; i < MAX_THREAD_NUM; i++)
        {
            results[current - MAX_THREAD_NUM + i] = futures[i].get();
        }
        left -= MAX_THREAD_NUM;
    }
    if (left > 0)
    {
        std::vector<std::future<PlanResult>> futures(left);
        for (int i = 0; i < left; i++)
        {
            futures[i] = std::async(
                std::launch::async,
                std::bind(&CarryPlanner::planThread, this, std::ref(inputs[current]))
            );
            current++;
        }
        for (int i = 0; i < left; i++)
        {
            results[current - left + i] = futures[i].get();
        }
    }

    int optimalIndex = -1;
    double optimalCost = std::numeric_limits<double>::max();
    for (int i = 0; i < numAnchor; i++)
    {
        if (results[i].success && results[i].gCost < optimalCost)
        {
            optimalIndex = i;
            optimalCost = results[i].gCost;
        }
    }
    if (optimalIndex == -1)
    {
        return PlanResult();
    }
    return results[optimalIndex];
}

PlanResult CarryPlanner::planThread(PlanThreadInput &input)
{
    // std::cout << "Thread " << std::this_thread::get_id() << " start" << std::endl;
    ElementMap threadMap = map;
    std::thread::id threadId = std::this_thread::get_id();
    PlanResult result;
    HybridAStar hybridAStar(threadMap, gridMap, distanceMap);

    /// step 1:
    TaskLowLevel task;
    task.start = input.agentPosition0;
    task.theta0 = input.agentTheta0;
    task.goal = input.agentPosition1;
    task.theta1 = input.agentTheta1;
    task.V = input.V;
    task.W = input.W;
    task.dt = input.dt;
    PlanResult result0 = hybridAStar.plan(task);
    std::cout << "[Thread " << threadId << "] " << "[0] success: " << result0.success << " gCost: " << result0.gCost << " timeCost: " << result0.timeCost << " iterations: " << result0.iterations << std::endl;
    if (!result0.success)
    {
        return PlanResult();
    }
    result.success = true;
    result.gCost += result0.gCost;
    result.timeCost += result0.timeCost;
    result.iterations += result0.iterations;
    result.actions.insert(result.actions.end(), result0.actions.begin(), result0.actions.end());

    /// step 2:
    auto targetObjectIt = std::find_if(
        threadMap.elements.begin(),
        threadMap.elements.end(),
        [input](Element &element) { return element.id == input.elementId; }
    );
    auto targetObject = *targetObjectIt;
    threadMap.elements.erase(targetObjectIt);

    Polygon unionPoly = unionPolygon(targetObject.shape, threadMap.agent.shape);
    Element agentBackup = threadMap.agent;
    threadMap.agent.shape = unionPoly;
    threadMap.agent.originAnchors.clear();
    threadMap.agent.anchors.clear();
    std::vector<CoordD> newOriginShape;
    double sint = std::sin(agentBackup.rotation);
    double cost = std::cos(agentBackup.rotation);
    double px = agentBackup.panning.x;
    double py = agentBackup.panning.y;
    for (int i = 0; i < unionPoly.vertices.size(); i++)
    {
        double x_ = unionPoly.vertices[i].x;
        double y_ = unionPoly.vertices[i].y;
        newOriginShape.push_back(
            CoordD(
                x_ * cost + y_ * sint - px * cost - py * sint,
                -x_ * sint + y_ * cost + px * sint - py * cost
            )
        );
        // std::cout << "(" << x_ << ", " << y_ << ")" << std::endl;
    }
    threadMap.agent.originShape = Polygon(newOriginShape);
    threadMap.agent.setGeometry(agentBackup.panning, agentBackup.rotation);
    // for (int i = 0; i < unionPoly.vertices.size(); i++)
    // {
    //     std::cout << "(" << threadMap.agent.shape.vertices[i].x << ", " << threadMap.agent.shape.vertices[i].y << ")" << std::endl;
    // }
    
    hybridAStar.ignoreId.push_back(input.elementId);
    hybridAStar.map = threadMap;
    task.start = input.agentPosition1;
    task.theta0 = input.agentTheta1;
    task.goal = input.agentPosition2;
    task.theta1 = input.agentTheta2;
    PlanResult result1 = hybridAStar.plan(task);
    std::cout << "[Thread " << threadId << "] " << "[1] success: " << result1.success << " gCost: " << result1.gCost << " timeCost: " << result1.timeCost << " iterations: " << result1.iterations << std::endl;

    if (!result1.success)
    {
        return PlanResult();
    }
    result.gCost += result1.gCost;
    result.timeCost += result1.timeCost;
    result.iterations += result1.iterations;
    result.actions.insert(result.actions.end(), result1.actions.begin(), result1.actions.end());

    /// step 3:
    threadMap.elements.push_back(targetObject);
    threadMap.agent = agentBackup;
    task.start = input.agentPosition2;
    task.theta0 = input.agentTheta2;
    task.goal = input.agentPosition3;
    task.theta1 = input.agentTheta3;

    hybridAStar.map = threadMap;
    hybridAStar.ignoreId.clear();

    PlanResult result2 = hybridAStar.plan(task);
    std::cout << "[Thread " << threadId << "] " << "[2] success: " << result2.success << " gCost: " << result2.gCost << " timeCost: " << result2.timeCost << " iterations: " << result2.iterations << std::endl;
    if (!result2.success)
    {
        return PlanResult();
    }
    result.gCost += result2.gCost;
    result.timeCost += result2.timeCost;
    result.iterations += result2.iterations;
    result.actions.insert(result.actions.end(), result2.actions.begin(), result2.actions.end());

    return result;
}

std::vector<CoordD> CarryPlanner::generateTargetPositions(Element element, CoordD &targetPosition, double &targetTheta)
{
    element.setGeometry(targetPosition, targetTheta);
    Element agent = map.agent;
    double d = std::sqrt(agent.originAnchors[0].x * agent.originAnchors[0].x + agent.originAnchors[0].y * agent.originAnchors[0].y);
    std::vector<CoordD> targetPositions(element.anchors.size());
    for (int i = 0; i < element.anchors.size(); i++)
    {
        double theta = std::atan2(element.panning.y - element.anchors[i].y, element.panning.x - element.anchors[i].x) + M_PI;
        CoordD move = CoordD(d * std::cos(theta), d * std::sin(theta)) + element.anchors[i];
        theta -= M_PI;
        agent.setGeometry(move, theta);
        targetPositions[i] = CoordD(agent.panning.x, agent.panning.y, agent.rotation);
    }
    return targetPositions;
}
