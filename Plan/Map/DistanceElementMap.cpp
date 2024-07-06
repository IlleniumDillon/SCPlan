#include "Map.hpp"

DistanceElementMap::DistanceElementMap()
{
    distanceMap = nullptr;
    resolution = 0;
    size = CoordI();
    minX = 0;
    minY = 0;
    maxX = 0;
    maxY = 0;
}

DistanceElementMap::DistanceElementMap(ElementMap &map, double resolution)
{
    this->resolution = resolution;
    this->minX = map.minX;
    this->minY = map.minY;
    this->maxX = map.maxX;
    this->maxY = map.maxY;
    double width = map.maxX - map.minX;
    double height = map.maxY - map.minY;
    size = CoordI(width / resolution, height / resolution);
    distanceMap = new DistanceElement *[size.x];
    for (int i = 0; i < size.x; i++)
    {
        distanceMap[i] = new DistanceElement[size.y];
        for (int j = 0; j < size.y; j++)
        {
            distanceMap[i][j] = DistanceElement();
        }
    }

    double rExternal = 0;
    for (auto &p : map.agent.originShape.vertices)
    {
        double r = p.norm();
        if (r > rExternal)
        {
            rExternal = r;
        }
    }
    rExternal *= 1.1;

    for (double x = minX; x < maxX; x += resolution)
    {
        for (double y = minY; y < maxY; y += resolution)
        {
            CoordD position(x, y);
            for (Element &e : map.elements)
            {
                if (e.shape.inside(position))
                {
                    for (double ix = x - rExternal; ix < x + rExternal; ix += resolution)
                    {
                        for (double iy = y - rExternal; iy < y + rExternal; iy += resolution)
                        {
                            DistanceElement *element = (*this)(ix, iy);
                            if (element == nullptr)
                            {
                                continue;
                            }
                            double d = (position - CoordD(ix, iy)).norm();
                            element->elementIds.insert(e.id);
                            if (d < element->distance)
                            {
                                element->distance = d;
                            }
                        }
                    }
                }
            }
        }
    }
}

DistanceElementMap::~DistanceElementMap()
{
    if (distanceMap == nullptr)
    {
        return;
    }
    for (int i = 0; i < size.x; i++)
    {
        delete[] distanceMap[i];
    }
    delete[] distanceMap;
    distanceMap = nullptr;
}

void DistanceElementMap::updateElements(ElementMap &map)
{
    double rExternal = 0;
    for (auto &p : map.agent.originShape.vertices)
    {
        double r = p.norm();
        if (r > rExternal)
        {
            rExternal = r;
        }
    }
    rExternal *= 1.1;

    for (double x = minX; x < maxX; x += resolution)
    {
        for (double y = minY; y < maxY; y += resolution)
        {
            CoordD position(x, y);
            DistanceElement *tmp = (*this)(x, y);
            if (tmp != nullptr)
            {
                tmp->elementIds.clear();
            }
            for (Element &e : map.elements)
            {
                if (e.shape.inside(position))
                {
                    for (double ix = x - rExternal; ix < x + rExternal; ix += resolution)
                    {
                        for (double iy = y - rExternal; iy < y + rExternal; iy += resolution)
                        {
                            DistanceElement *element = (*this)(ix, iy);
                            if (element == nullptr)
                            {
                                continue;
                            }
                            double d = (position - CoordD(ix, iy)).norm();
                            element->elementIds.insert(e.id);
                            if (d < element->distance)
                            {
                                element->distance = d;
                            }
                        }
                    }
                }
            }
        }
    }
}

DistanceElementMap &DistanceElementMap::operator=(const DistanceElementMap &distanceElementMap)
{
    if (distanceMap != nullptr)
    {
        for (int i = 0; i < size.x; i++)
        {
            delete[] distanceMap[i];
        }
        delete[] distanceMap;
    }
    size = distanceElementMap.size;
    resolution = distanceElementMap.resolution;
    minX = distanceElementMap.minX;
    minY = distanceElementMap.minY;
    maxX = distanceElementMap.maxX;
    maxY = distanceElementMap.maxY;
    distanceMap = new DistanceElement *[size.x];
    for (int i = 0; i < size.x; i++)
    {
        distanceMap[i] = new DistanceElement[size.y];
        for (int j = 0; j < size.y; j++)
        {
            distanceMap[i][j] = distanceElementMap.distanceMap[i][j];
        }
    }
    return *this;
}

DistanceElement *DistanceElementMap::operator[](CoordI &index)
{
    return (*this)(index.x, index.y);
}

DistanceElement *DistanceElementMap::operator[](CoordD &position)
{
    return (*this)(position.x, position.y);
}

DistanceElement *DistanceElementMap::operator()(int x, int y)
{
    if (x < 0 || x >= size.x || y < 0 || y >= size.y)
    {
        return nullptr;
    }
    return &distanceMap[x][y];
}

DistanceElement *DistanceElementMap::operator()(double x, double y)
{
    int i = (x - minX) / resolution;
    int j = (y - minY) / resolution;
    return (*this)(i, j);
}
