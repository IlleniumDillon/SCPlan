#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <bitset>
#include <vector>

class VertexProperties
{
public:
    int domain_id;
};

class EdgeProperties
{
public:
    bool connected;
};

class UndirectedGraph
{
public:
    std::vector<VertexProperties> vertex_properties;
    std::vector<std::vector<EdgeProperties>> edge_properties;
public:
    // UndirectedGraph();

    // void add_vertex();
    // void add_edge();
    // void remove_vertex();
    // void remove_edge();
    // VertexProperties* get_vertex();
    // EdgeProperties* get_edge();
    // int get_vertex_index();
    // std::pair<int, int> get_edge_index();
    // std::vector<EdgeProperties> get_out_degree();
};



#endif // GRAPH_HPP