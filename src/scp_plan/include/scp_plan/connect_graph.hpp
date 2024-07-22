#ifndef CONNECT_GRAPH_HPP
#define CONNECT_GRAPH_HPP

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <set>

#include "graph_typedef.hpp"
#include "grid_map.hpp"
#include "pix_map.hpp"

#include <opencv2/opencv.hpp>

namespace scp
{

class ConnectNode
{
public:
    ConnectNode();
    ConnectNode(int index, Point position);
public:
    int index;
    Point position;
    std::vector<int> edges;

    double g = std::numeric_limits<double>::max();
    std::multimap<double, int>::iterator iterator;
    int parent = -1;
    int edge = -1;
    int state = NOT_VISITED;
public:

};

class ConnectEdge
{
public:
    ConnectEdge();
    ConnectEdge(int id, int index1, int index2, double weight);
public:
    int id;
    int index1;
    int index2;
    double weight;
public:
    int getOtherIndex(int index);
};

class ConnectRoute
{
public:
    ConnectRoute(){}
    ConnectRoute(Point start, Point end, int id)
    {
        this->start = start;
        this->end = end;
        this->id = id;
    }
public:
    Point start;
    int id;
    Point end;
};

typedef std::vector<ConnectRoute> ConnectRoutes;

class ConnectGraph
{
public:
    PixMap pix_map;
    double radius = 0.25;

    std::vector<ConnectNode> nodes;
    std::vector<ConnectEdge> edges; 
public:
    ConnectGraph();
public:
    void config(double radius);
    void convertGridMap(GridMap& grid_map);
    void buildGraph(std::vector<Element>& dynamic_elements);

    void dijkstra(Point start, Point goal, ConnectRoutes& path);
};
} // namespace scp

#endif // CONNECT_GRAPH_HPP