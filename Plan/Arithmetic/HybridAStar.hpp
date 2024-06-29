#ifndef HYBRIDASTAR_HPP
#define HYBRIDASTAR_HPP

#include <iostream>
#include <cmath>
#include <chrono>
#include "Map.hpp"
#include "Kinematics.hpp"
#include "Common.hpp"

class HybridAStar
{
public:
    ElementMap map;
    std::vector<Kinematics> kinematics;
    TaskLowLevel task;
    GridMap gridMap;
    DistanceElementMap distanceMap;

    std::multimap<double, Grid*> openSet;
    std::vector<Grid*> closeSet;

    std::vector<int> ignoreId;
public:
    HybridAStar(){}
    HybridAStar(ElementMap &map, double solveResolution = 0.05, double distanceMapResolution = 0.5);
    HybridAStar(ElementMap &map, GridMap &gridMap, DistanceElementMap &distanceMap);
    void initGridMap(ElementMap &map, double solveResolution = 0.05);
    void initDistanceElementMap(ElementMap &map, double distanceMapResolution = 0.5);
    void updateAllMaps(ElementMap &map, GridMap &gridMap, DistanceElementMap &distanceMap);
    void resetMap();
    void setIgnoreId(std::vector<int> &ignoreId);   

    void generateKinematics();
    PlanResult plan(TaskLowLevel &task);
    void getNeighbors(Grid* current, std::vector<Grid*> &neighbors, std::vector<Action> &actions, std::vector<double> &costs);
    double heuristic(Grid* current, Grid* goal, double weight = 1.0);
    bool checkCollision(Element& element);
    std::vector<Action> finalPath(CoordD &position0, double &theta0, CoordD &position1, double &theta1);
};

#endif // HYBRIDASTAR_HPP