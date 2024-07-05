#include "Polygon.hpp"

Polygon::Polygon(const std::vector<CoordD> &vertices)
{
    this->vertices = vertices;
    for (int i = 0; i < vertices.size(); i++)
    {
        edges.push_back(Line(vertices[i], vertices[(i + 1) % vertices.size()]));
    }
}

bool Polygon::inside(CoordD &coord)
{
    Line ray(coord, CoordD(1e9, coord.y));
    int count = 0;
    for (Line &edge : edges)
    {
        if (edge.isOnLine(coord))
        {
            return true;
        }
        if (edge.isIntersect(ray))
        {
            count++;
        }
    }
    return count % 2 == 1;
}

bool Polygon::isIntersect(Line &line)
{
    for (Line &edge : edges)
    {
        if (edge.isIntersect(line))
        {
            return true;
        }
    }
    return false;
}

bool Polygon::isIntersect(Polygon &polygon)
{
    for (Line &edge : edges)
    {
        if (polygon.isIntersect(edge))
        {
            return true;
        }
    }
    return false;
}

Polygon bunghole(std::vector<CoordD> &points)
{
    /// Andrew's Monotone Chain
    auto point3Cross = [](const CoordD& O, const CoordD& A, const CoordD& B) -> double
    {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    };
    int n = points.size(), k = 0;
    if (n <= 3) return Polygon(points);
    std::vector<CoordD> H(2*n);

    std::sort(points.begin(), points.end(), [](CoordD& p1, CoordD& p2){
        return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
    });

    for (int i = 0; i < n; i++)
    {
        while (k >= 2 && point3Cross(H[k-2],H[k-1],points[i]) <= 0) k--;
        H[k++] = points[i];
    }

    for (int i = n-2, t = k+1; i >= 0; i--)
    {
        while (k >= t && point3Cross(H[k-2],H[k-1],points[i]) <= 0) k--;
        H[k++] = points[i];
    }

    H.resize(k-1);

    return Polygon(H);
}

Polygon unionPolygon(Polygon &p1, Polygon &p2)
{
    std::vector<CoordD> points;
    points.insert(points.end(),p1.vertices.begin(),p1.vertices.end());
    points.insert(points.end(),p2.vertices.begin(),p2.vertices.end());
    return bunghole(points);
}

Polygon unionPolygon(std::vector<Polygon> &polygons)
{
    std::vector<CoordD> points;
    for (auto& e:polygons)
    {
        points.insert(points.end(),e.vertices.begin(),e.vertices.end());
    }
    return bunghole(points);
}
