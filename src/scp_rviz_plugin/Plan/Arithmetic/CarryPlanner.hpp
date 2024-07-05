#ifndef CARRY_PLANNER_HPP
#define CARRY_PLANNER_HPP

#include "HybridAStar.hpp"

/*
计算搬运每个锚点时无人车的位姿，和物体处于目标位姿时无人车的位姿
根据被搬运物体的锚点数量新建线程，限制最多4线程，需要更多线程的话要另一个批次。
每个线程搜索一种情况，将所有结果汇总，选出代价最小的作为结果。

线程的输入是elementMap,无人车起始位置、姿态，搬物体的前置位置、姿态，搬物体的目标位置、姿态，被搬运物体的编号
搜索过程是调用HybridAStar先转移到搬物体的前置位姿，如果无解，直接返回；
如果有解，将被搬运物体从elements中移除，与无人车整合成一个新的对象，调用HybridAStar搜索到搬物体的目标位姿。

可能要实现的功能，gridmap和distanceMap的复用，不用每次都新建。
*/
#define MAX_THREAD_NUM (4)

class CarryPlanner
{
public:
    ElementMap map;
    GridMap gridMap;
    DistanceElementMap distanceMap;
    TaskCarry task;

public:
    CarryPlanner(){}
    CarryPlanner(ElementMap &map, double solveResolution = 0.05, double distanceMapResolution = 0.5);

    PlanResult plan(TaskCarry &task);

private:
    class PlanThreadInput
    {
    public:
        int elementId;

        CoordD agentPosition0;
        double agentTheta0;

        CoordD agentPosition1;
        double agentTheta1;

        CoordD agentPosition2;
        double agentTheta2;

        CoordD agentPosition3;
        double agentTheta3;

        double V, W;
        double dt;
    };
    
    PlanResult planThread(PlanThreadInput& input);
    std::vector<CoordD> generateTargetPositions(Element element, CoordD &targetPosition, double& targetTheta);
};

#endif // CARRY_PLANNER_HPP