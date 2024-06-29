#ifndef TEST_HPP
#define TEST_HPP

#include <iostream>
#include "Coordinate.hpp"
#include "Line.hpp"
#include "Polygon.hpp"
#include "Shape.hpp"
#include "Kinematics.hpp"
#include "Map.hpp"
#include "Visualize.hpp"
#include "HybridAStar.hpp"
#include "CarryPlanner.hpp"

#ifndef MAPFILE_PATH
#error "MAPFILE_PATH is not defined"
#endif
#ifndef CONFIG_PATH
#error "CONFIG_PATH is not defined"
#endif
#ifndef TASK_PATH
#error "TASK_PATH is not defined"
#endif

class ResolutionConfig
{
public:
    double solveResolution = 0.05;
    double distanceMapResolution = 0.5;
};

ResolutionConfig loadConfig(const std::string &path);
TaskLowLevel loadTask(const std::string &path);
TaskCarry loadTaskCarry(const std::string &path);

void MannualTest();
void HybridAStarPlan();
void CarryPlan();

#endif //TEST_HPP