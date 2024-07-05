#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include <iostream>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "Map.hpp"
#include "Common.hpp"

void draw(ElementMap &map, std::string windowName = "Map");
void draw(ElementMap &map, PlanResult &result, std::string windowName = "Map");
void drawDynamic(ElementMap &map, PlanResult &result, std::string windowName = "Map");

#endif // VISUALIZE_HPP