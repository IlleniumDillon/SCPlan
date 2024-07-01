#ifndef LINE_HPP
#define LINE_HPP

#include "Coordinate.hpp"

#define DOUBLE_TOLERANCE 0.05

/// use 3D coordinate system, but ignore z
class Line
{
public:
    Coord<double> start, end;
public:
    Line(const Coord<double> &start, const Coord<double> &end);
    bool isIntersect(const Line &line) const;
    Coord<double> getIntersect(const Line &line) const;
    bool isParallel(const Line &line) const;
    bool isOnLine(Coord<double> coord);

    std::vector<Coord<double>> discretize(double resolution) const;
};

#endif // LINE_HPP
