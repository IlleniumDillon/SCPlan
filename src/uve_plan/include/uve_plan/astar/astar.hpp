#ifndef UVE_PLAN_ASTAR_HPP
#define UVE_PLAN_ASTAR_HPP


#include "uvs_tools/graph_search/graph_search.hpp"
#include <iostream>

constexpr int GRAPH_DIMENSION = 2;
class AStarNode : public GraphNodeBase<GRAPH_DIMENSION>
{
public:
    bool occupied = false;
};
class AStarGraph : public GridGraph<AStarNode, GRAPH_DIMENSION>
{
public:
    AStarGraph() : GridGraph<AStarNode, GRAPH_DIMENSION>(){}
    AStarGraph(Eigen::Matrix<double, 1, GRAPH_DIMENSION>& size, Eigen::Matrix<double, 1, GRAPH_DIMENSION>& resolution, Eigen::Matrix<double, 1, GRAPH_DIMENSION>& origin)
        : GridGraph<AStarNode, GRAPH_DIMENSION>(size, resolution, origin)
        {}
    AStarGraph(Eigen::Matrix<int, 1, GRAPH_DIMENSION>& grid_size, Eigen::Matrix<double, 1, GRAPH_DIMENSION>& resolution, Eigen::Matrix<double, 1, GRAPH_DIMENSION>& origin)
    {
        this->size = grid_size.cast<double>().cwiseProduct(resolution);
        this->grid_size = grid_size;
        this->resolution = resolution;
        this->origin = origin;
        this->nodes.resize(grid_size[0] * grid_size[1]);
        for (int i = 0; i < nodes.size(); i++)
        {
            nodes[i] = new AStarNode();
            
            for (int j = 0; j < GRAPH_DIMENSION; j++)
            {
                nodes[i]->index(j) = i / grid_size.block(0, 0, 1, j).prod() % grid_size(j);
                nodes[i]->state(j) = origin(j) + nodes[i]->index(j) * resolution(j);
            }

        }
    }
};
using AStarTraceType = Eigen::Matrix<double, 1, GRAPH_DIMENSION>;
class AStarResult : public GraphSearchResultBase<AStarTraceType>{};
class AStarSearch : public GraphSearchBase<AStarGraph, AStarNode, AStarResult>
{
public:
    virtual void updateGraph(AStarGraph& graph)
    {
        this->graph = graph;
    }
    virtual void reset()
    {
        for (auto p : openSet)
        {
            ((AStarNode*)p.second)->flag = NOT_VISITED;
        }
        for (auto p : closeSet)
        {
            p->flag = NOT_VISITED;
        }
        openSet.clear();
        closeSet.clear();
        result = AStarResult();
    }
    virtual GraphNodeCost heuristic(AStarNode* node1, AStarNode* node2)
    {
        return GraphNodeCost((node1->state - node2->state).norm(), 0);
    }
    virtual void getNeighbors(AStarNode* node, std::vector<AStarNode*>& neighbors, std::vector<GraphNodeCost>& costs)
    {
        const Eigen::Matrix<int, 1, GRAPH_DIMENSION> dir[] = 
        {
            Eigen::Matrix<int, 1, GRAPH_DIMENSION>(1, 0),
            Eigen::Matrix<int, 1, GRAPH_DIMENSION>(-1, 0),
            Eigen::Matrix<int, 1, GRAPH_DIMENSION>(0, 1),
            Eigen::Matrix<int, 1, GRAPH_DIMENSION>(0, -1),
            Eigen::Matrix<int, 1, GRAPH_DIMENSION>(1, 1),
            Eigen::Matrix<int, 1, GRAPH_DIMENSION>(1, -1),
            Eigen::Matrix<int, 1, GRAPH_DIMENSION>(-1, 1),
            Eigen::Matrix<int, 1, GRAPH_DIMENSION>(-1, -1)
        };
        neighbors.clear();
        costs.clear();
        // std::cout << "node: " << node->index << std::endl;
        for (int i = 0; i < 8; i++)
        {
            // std::cout << "node: " << node->index + dir[i] << std::endl;
            auto neighbor = graph[node->index + dir[i]];
            if (neighbor != nullptr && !neighbor->occupied && neighbor->flag != IN_CLOSESET)
            {
                neighbors.push_back(neighbor);
                costs.push_back(GraphNodeCost(dir[i].cast<double>().norm(), 0));
            }
        }
        // std::cout << "neighbors: " << neighbors.size() << std::endl;
    }
};

#endif // UVE_PLAN_ASTAR_HPP