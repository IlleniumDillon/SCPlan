#include "element.hpp"

using namespace scp;

scp::Element::Element(int id, std::vector<Point> originVertices, std::vector<Point> originAnchors, double collisionRadius)
    : id(id), originVertices(originVertices), originAnchors(originAnchors)
{
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;

    currentVertices = originVertices;
    currentAnchors = originAnchors;
    if (collisionRadius > 0)
    {
        Point center;
        center.x = 0;
        center.y = 0;
        for (int i = 0; i < originVertices.size(); i++)
        {
            center.x += originVertices[i].x;
            center.y += originVertices[i].y;
        }
        center.x /= originVertices.size();
        center.y /= originVertices.size();
        for (int i = 0; i < originVertices.size(); i++)
        {
            double x = originVertices[i].x;
            double y = originVertices[i].y;
            Point vertex;
            double theta = atan2(y - center.y, x - center.x);
            vertex.x = x + collisionRadius * cos(theta);
            vertex.y = y + collisionRadius * sin(theta);
            originCollisionVertices.push_back(vertex);
        }
        currentCollisionVertices = originCollisionVertices;
    }
    else
    {
        originCollisionVertices.clear();
        currentCollisionVertices.clear();
    }
}

void scp::Element::updatePose(Pose2D pose)
{
    this->pose = pose;
    currentVertices = originVertices;
    currentAnchors = originAnchors;
    for (int i = 0; i < currentVertices.size(); i++)
    {
        double x = currentVertices[i].x;
        double y = currentVertices[i].y;
        currentVertices[i].x = x * cos(pose.theta) - y * sin(pose.theta) + pose.x;
        currentVertices[i].y = x * sin(pose.theta) + y * cos(pose.theta) + pose.y;
    }
    for (int i = 0; i < currentAnchors.size(); i++)
    {
        double x = currentAnchors[i].x;
        double y = currentAnchors[i].y;
        currentAnchors[i].x = x * cos(pose.theta) - y * sin(pose.theta) + pose.x;
        currentAnchors[i].y = x * sin(pose.theta) + y * cos(pose.theta) + pose.y;
    }
    for (int i = 0; i < originCollisionVertices.size(); i++)
    {
        double x = originCollisionVertices[i].x;
        double y = originCollisionVertices[i].y;
        currentCollisionVertices[i].x = x * cos(pose.theta) - y * sin(pose.theta) + pose.x;
        currentCollisionVertices[i].y = x * sin(pose.theta) + y * cos(pose.theta) + pose.y;
    }
}

void scp::Element::getDispersed(std::vector<std::pair<int, int>> &dispersed, double resolution, int oriX, int oriY)
{
    dispersed.clear();

    if (currentVertices.size() == 0)
        return;

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    std::vector<cv::Point2f> vertices;
    for (int i = 0; i < currentVertices.size(); i++)
    {
        vertices.push_back(cv::Point2f(currentVertices[i].x, currentVertices[i].y));
        minX = std::min(minX, currentVertices[i].x);
        minY = std::min(minY, currentVertices[i].y);
        maxX = std::max(maxX, currentVertices[i].x);
        maxY = std::max(maxY, currentVertices[i].y);
    }

    int minI = (int)(minX / resolution);
    int minJ = (int)(minY / resolution);
    int maxI = (int)(maxX / resolution);
    int maxJ = (int)(maxY / resolution);

    for (int i = minI; i <= maxI; i++)
    {
        for (int j = minJ; j <= maxJ; j++)
        {
            double x = i * resolution;
            double y = j * resolution;
            if (cv::pointPolygonTest(vertices, cv::Point2f(x, y), false) >= 0)
            {
                dispersed.push_back(std::make_pair(i - oriX, j - oriY));
            }
        }
    }
}

void scp::Element::getCollision(std::vector<std::pair<int, int>> &collision, double resolution, int oriX, int oriY)
{
    collision.clear();

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    std::vector<cv::Point2f> vertices;
    for (int i = 0; i < currentCollisionVertices.size(); i++)
    {
        vertices.push_back(cv::Point2f(currentCollisionVertices[i].x, currentCollisionVertices[i].y));
        minX = std::min(minX, currentCollisionVertices[i].x);
        minY = std::min(minY, currentCollisionVertices[i].y);
        maxX = std::max(maxX, currentCollisionVertices[i].x);
        maxY = std::max(maxY, currentCollisionVertices[i].y);
    }

    int minI = (int)(minX / resolution);
    int minJ = (int)(minY / resolution);
    int maxI = (int)(maxX / resolution);
    int maxJ = (int)(maxY / resolution);

    for (int i = minI; i <= maxI; i++)
    {
        for (int j = minJ; j <= maxJ; j++)
        {
            double x = i * resolution;
            double y = j * resolution;
            if (cv::pointPolygonTest(vertices, cv::Point2f(x, y), false) >= 0)
            {
                collision.push_back(std::make_pair(i - oriX, j - oriY));
            }
        }
    }
}

bool lineIntersection(Point p1, Point p2, Point p3, Point p4)
{
    if (
        std::min(p1.x, p2.x) > std::max(p3.x, p4.x) ||
        std::min(p3.x, p4.x) > std::max(p1.x, p2.x) ||
        std::min(p1.y, p2.y) > std::max(p3.y, p4.y) ||
        std::min(p3.y, p4.y) > std::max(p1.y, p2.y)
    )
    {
        return false;
    }
    double u,v,w,z;
    u = (p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x);
    v = (p4.x - p1.x) * (p2.y - p1.y) - (p4.y - p1.y) * (p2.x - p1.x);
    w = (p1.x - p3.x) * (p4.y - p3.y) - (p1.y - p3.y) * (p4.x - p3.x);
    z = (p2.x - p3.x) * (p4.y - p3.y) - (p2.y - p3.y) * (p4.x - p3.x);
    return u * v <= 0 && w * z <= 0;
}

bool scp::Element::isCollision(Element &other)
{
    for (int i = 0; i < currentVertices.size(); i++)
    {
        Point p1 = currentVertices[i];
        Point p2 = currentVertices[(i + 1) % currentVertices.size()];
        for (int j = 0; j < other.currentVertices.size(); j++)
        {
            Point p3 = other.currentVertices[j];
            Point p4 = other.currentVertices[(j + 1) % other.currentVertices.size()];
            if (lineIntersection(p1, p2, p3, p4))
            {
                return true;
            }
        }
    }
    return false;
}
