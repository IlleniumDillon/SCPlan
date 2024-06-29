#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "Coordinate.hpp"
#include "Line.hpp"

class Polygon
{
public:
    std::vector<CoordD> vertices;
    std::vector<Line> edges;
public:
    Polygon(const std::vector<CoordD> &vertices);
    Polygon(){}
    
    bool inside(CoordD &coord);
    bool isIntersect(Line &line);
    bool isIntersect(Polygon &polygon);
};

Polygon bunghole(std::vector<CoordD> &points);
Polygon unionPolygon(Polygon &p1, Polygon &p2);
Polygon unionPolygon(std::vector<Polygon> &polygons);

#endif // POLYGON_HPP