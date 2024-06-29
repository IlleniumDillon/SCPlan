#include "Line.hpp"

Line::Line(const Coord<double> &start, const Coord<double> &end)
{
    this->start = start;
    this->end = end;
    if (this->start.coordSystem != this->end.coordSystem)
    {
        this->start = this->start.transform(&globalCoordSystem);
        this->end = this->end.transform(&globalCoordSystem);
    }
}

bool Line::isIntersect(const Line &line) const
{
    if(
        !(
            std::min(start.x,end.x)<=std::max(line.start.x,line.end.x) && 
            std::min(line.start.y,line.end.y)<=std::max(start.y,end.y) &&
            std::min(line.start.x,line.end.x)<=std::max(start.x,end.x) && 
            std::min(start.y,end.y)<=std::max(line.start.y,line.end.y)
        )
    )
    return false;
    double u,v,w,z;
    u=(line.start.x-start.x)*(end.y-start.y)-(end.x-start.x)*(line.start.y-start.y);
    v=(line.end.x-start.x)*(end.y-start.y)-(end.x-start.x)*(line.end.y-start.y);
    w=(start.x-line.start.x)*(line.end.y-line.start.y)-(line.end.x-line.start.x)*(start.y-line.start.y);
    z=(end.x-line.start.x)*(line.end.y-line.start.y)-(line.end.x-line.start.x)*(end.y-line.start.y);
    return (u*v<=0 && w*z<=0);
}

Coord<double> Line::getIntersect(const Line &line) const
{
    if (isIntersect(line))
    {
        double a1 = end.y - start.y;
        double b1 = start.x - end.x;
        double c1 = a1 * start.x + b1 * start.y;

        double a2 = line.end.y - line.start.y;
        double b2 = line.start.x - line.end.x;
        double c2 = a2 * line.start.x + b2 * line.start.y;

        double det = a1 * b2 - a2 * b1;
        double x = (b2 * c1 - b1 * c2) / det;
        double y = (a1 * c2 - a2 * c1) / det;

        return Coord<double>(x, y, 0, &globalCoordSystem);
    }
    return Coord<double>(0, 0, 0, nullptr);
}

bool Line::isParallel(const Line &line) const
{
    double a1 = end.y - start.y;
    double b1 = start.x - end.x;
    double c1 = a1 * start.x + b1 * start.y;

    double a2 = line.end.y - line.start.y;
    double b2 = line.start.x - line.end.x;
    double c2 = a2 * line.start.x + b2 * line.start.y;

    double det = a1 * b2 - a2 * b1;
    return abs(det) < DOUBLE_TOLERANCE;
}

bool Line::isOnLine(Coord<double> coord)
{
    if (coord.coordSystem != start.coordSystem)
    {
        coord = coord.transform(start.coordSystem);
    }
    if (coord.x < std::min(start.x, end.x) || 
        coord.x > std::max(start.x, end.x) || 
        coord.y < std::min(start.y, end.y) || 
        coord.y > std::max(start.y, end.y))
    {
        return false;
    }
    Line line1 = Line(start, coord);
    Line line2 = Line(coord, end);
    return line1.isParallel(line2);
}

std::vector<Coord<double>> Line::discretize(double resolution) const
{
    std::vector<Coord<double>> result;

    Coord<double> direction = end - start;
    double length = direction.norm();
    direction /= length;

    Coord<double> current = start;
    while ((end - current).norm() > resolution)
    {
        result.push_back(current);
        current += direction * resolution;
    }
    result.push_back(end);

    return result;
}
