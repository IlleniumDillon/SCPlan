#include "Visualize.hpp"

void draw(ElementMap &map, std::string windowName)
{
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    int targetWidth = 900;
    int targetHeight = 900;

    double scaleX = targetWidth / (map.maxX - map.minX);
    double scaleY = targetHeight / (map.maxY - map.minY);
    double scale = std::min(scaleX, scaleY);

    targetHeight = (map.maxY - map.minY) * scale;
    targetWidth = (map.maxX - map.minX) * scale;

    cv::Mat img(targetHeight, targetWidth, CV_8UC3, cv::Scalar(255, 255, 255));

    // std::vector<std::vector<cv::Point>> contours;
    for (Element &e : map.elements)
    {
        std::vector<cv::Point> points;
        for (CoordD &c : e.shape.vertices)
        {
            CoordD p = c.transform(&map.visualizeCS) * scale;
            points.push_back(cv::Point(p.x, p.y));
        }
        // contours.push_back(points);
        cv::fillConvexPoly(img, points, cv::Scalar(0, 0, 0));
    }
    // cv::fillPoly(img, contours, cv::Scalar(0, 0, 0));

    std::vector<cv::Point> points;
    for (CoordD &c : map.agent.shape.vertices)
    {
        CoordD p = c.transform(&map.visualizeCS) * scale;
        points.push_back(cv::Point(p.x, p.y));
    }
    cv::fillConvexPoly(img, points, cv::Scalar(0, 255, 0));

    cv::imshow(windowName, img);
    // cv::waitKey(0);
}

void draw(ElementMap &map, PlanResult &result, std::string windowName)
{
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    int targetWidth = 900;
    int targetHeight = 900;

    double scaleX = targetWidth / (map.maxX - map.minX);
    double scaleY = targetHeight / (map.maxY - map.minY);
    double scale = std::min(scaleX, scaleY);

    targetHeight = (map.maxY - map.minY) * scale;
    targetWidth = (map.maxX - map.minX) * scale;

    cv::Mat img(targetHeight, targetWidth, CV_8UC3, cv::Scalar(255, 255, 255));

    // std::vector<std::vector<cv::Point>> contours;
    for (Element &e : map.elements)
    {
        std::vector<cv::Point> points;
        for (CoordD &c : e.shape.vertices)
        {
            CoordD p = c.transform(&map.visualizeCS) * scale;
            points.push_back(cv::Point(p.x, p.y));
        }
        // contours.push_back(points);
        cv::fillConvexPoly(img, points, cv::Scalar(0, 0, 0));
    }
    // cv::fillPoly(img, contours, cv::Scalar(0, 0, 0));

    std::vector<cv::Point> points;
    for (CoordD &c : map.agent.shape.vertices)
    {
        CoordD p = c.transform(&map.visualizeCS) * scale;
        points.push_back(cv::Point(p.x, p.y));
    }
    cv::fillConvexPoly(img, points, cv::Scalar(0, 255, 0));

    for (auto & a : result.actions)
    {
        CoordD p1 = a.from.transform(&map.visualizeCS) * scale;
        CoordD p2 = a.to.transform(&map.visualizeCS) * scale;
        // cv::line(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(0, 0, 255), 2);
        cv::circle(img, cv::Point(p1.x, p1.y), 2, cv::Scalar(0, 0, 255), -1);
        cv::circle(img, cv::Point(p2.x, p2.y), 2, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow(windowName, img);
    // cv::waitKey(0);
}

void drawDynamic(ElementMap &map, PlanResult &result, std::string windowName)
{
    for (auto &r : result.actions)
    {
        map.agent.setGeometry(r.to, r.theta1);
        draw(map, result, windowName);
        cv::waitKey(500);
    }
}
