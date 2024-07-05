#include "Map.hpp"

bool checkElementCollision(Element &e, std::vector<Element> &obstacles)
{
    for (Element &o : obstacles)
    {
        if (e.shape.isIntersect(o.shape))
            return true;
    }
    return false;
}

void genreateDistanceElementMap(ElementMap &map, double resolution, DistanceElementMap &distanceElementMap)
{
    distanceElementMap = DistanceElementMap(map, resolution);
}
