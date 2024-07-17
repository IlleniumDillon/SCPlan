#ifndef ELEMENT_HPP
#define ELEMENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <map>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <opencv4/opencv2/opencv.hpp>

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose2D;

namespace scp
{

class Element
{
public:
    int id;
    Pose2D pose;
    std::vector<Point> originVertices;
    std::vector<Point> currentVertices;
    std::vector<Point> originAnchors;
    std::vector<Point> currentAnchors;
    std::vector<Point> originCollisionVertices;
    std::vector<Point> currentCollisionVertices;
public:
    Element(){};
    Element(int id, std::vector<Point> originVertices, std::vector<Point> originAnchors, double collisionRadius);
    void updatePose(Pose2D pose);
    void getDispersed(std::vector<std::pair<int,int>> &dispersed, double resolution, int oriX, int oriY);
    void getCollision(std::vector<std::pair<int,int>> &collision, double resolution, int oriX, int oriY);
    bool isCollision(Element& other);
};

Element unionElement(Element& major, Element& minor);

} // namespace scp

#endif // ELEMENT_HPP