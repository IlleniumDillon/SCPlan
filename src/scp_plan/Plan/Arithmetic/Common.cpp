#include "Common.hpp"

void generateGridMap(ElementMap &map, double resolution, GridMap &gridMap)
{
    gridMap = GridMap(map, resolution);
}