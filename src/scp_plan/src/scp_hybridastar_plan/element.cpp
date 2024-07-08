#include "element.hpp"

using namespace scp;

scp::Element::Element(int id, std::vector<Point> originVertices, std::vector<Point> originAnchors)
    : id(id), originVertices(originVertices), originAnchors(originAnchors)
{
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;

    currentVertices = originVertices;
    currentAnchors = originAnchors;
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
}

void scp::Element::getDispersed(std::vector<std::pair<int, int>> &dispersed, double resolution, int oriX, int oriY)
{
    dispersed.clear();

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
