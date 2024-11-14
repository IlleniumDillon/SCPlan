#ifndef UVE_HYBRID_ASTAR_HPP
#define UVE_HYBRID_ASTAR_HPP

#include "uvs_tools/graph_search/graph_search.hpp"
#include <iostream>

constexpr int GRAPH_DIMENSION = 3;
class HybridAStarNode : public GraphNodeBase<GRAPH_DIMENSION>
{
public:
    bool occupied = false;
};
class HybridAStarGraph : public GridGraph<HybridAStarNode, GRAPH_DIMENSION>
{
public:
    HybridAStarGraph() : GridGraph<HybridAStarNode, GRAPH_DIMENSION>(){}
    HybridAStarGraph(Eigen::Matrix<double, 1, GRAPH_DIMENSION>& size, Eigen::Matrix<double, 1, GRAPH_DIMENSION>& resolution, Eigen::Matrix<double, 1, GRAPH_DIMENSION>& origin)
        : GridGraph<HybridAStarNode, GRAPH_DIMENSION>(size, resolution, origin)
        {}
    HybridAStarGraph(Eigen::Matrix<int, 1, GRAPH_DIMENSION>& grid_size, Eigen::Matrix<double, 1, GRAPH_DIMENSION>& resolution, Eigen::Matrix<double, 1, GRAPH_DIMENSION>& origin)
    {
        this->size = grid_size.cast<double>().cwiseProduct(resolution);
        this->grid_size = grid_size;
        this->resolution = resolution;
        this->origin = origin;
        this->nodes.resize(grid_size[0] * grid_size[1] * grid_size[2]);
        for (int i = 0; i < nodes.size(); i++)
        {
            nodes[i] = new HybridAStarNode();
            
            for (int j = 0; j < GRAPH_DIMENSION; j++)
            {
                nodes[i]->index(j) = i / grid_size.block(0, 0, 1, j).prod() % grid_size(j);
                nodes[i]->state(j) = origin(j) + nodes[i]->index(j) * resolution(j);
            }
        }
    }
    HybridAStarNode* operator[](Eigen::Matrix<double, 1, GRAPH_DIMENSION> state) 
    {
        Eigen::Matrix<int, 1, GRAPH_DIMENSION> index;
        for (int i = 0; i < GRAPH_DIMENSION; i++)
        {
            index(i) = (state(i) - origin(i)) / resolution(i);
        }
        return operator()(index);
    }
    HybridAStarNode* operator()(Eigen::Matrix<int, 1, GRAPH_DIMENSION> index) 
    {
        if ((index.minCoeff() < 0) || ((grid_size - index).minCoeff() <= 0))
        {
            return nullptr;
        }
        int i = index(GRAPH_DIMENSION - 1);
        for (int j = GRAPH_DIMENSION - 2; j >= 0; j--)
        {
            i = i * grid_size(j) + index(j);
        }
        return nodes[i];
    }
};
using HybridAStarTraceType = Eigen::Matrix<double, 1, 3>;
class HybridAStarResult : public GraphSearchResultBase<HybridAStarTraceType>{};
class HybridAStarSearch : public GraphSearchBase<HybridAStarGraph, HybridAStarNode, HybridAStarResult>
{
public:
    void generateNeighborList(double max_v, double max_w, int step_v, int step_w, double dt)
    {
        double v_ = max_v / step_v;
        double w_ = max_w / step_w;
        for (int i = -step_v; i <= step_v; i++)
        {
            for (int j = -step_w; j <= step_w; j++)
            {
                double v = i * v_;
                double w = j * w_;
                
                Eigen::Matrix<double, 1, 3> neighbor;
                neighbor(2) = w * dt;
                neighbor(1) = neighbor(2) / 2;
                if (j == 0)
                {
                    neighbor(0) = v * dt;
                }
                else
                {
                    double R = v / w;
                    double L = std::sqrt(2*R*R*(1 - std::cos(w * dt)));
                    neighbor(0) = L;
                }
                neighborList.push_back(neighbor);

                neighborCost.push_back(std::abs(v*dt));
                if (v < 0)
                {
                    neighborCost.back() *= 2;
                }
                else if (v == 0)
                {
                    neighborCost.back() = dt;
                }
            }
        }
    }
    virtual void updateGraph(HybridAStarGraph& graph)
    {
        this->graph = graph;
    }
    virtual void reset()
    {
        for (auto p : openSet)
        {
            ((HybridAStarNode*)p.second)->flag = NOT_VISITED;
        }
        for (auto p : closeSet)
        {
            p->flag = NOT_VISITED;
        }
        openSet.clear();
        closeSet.clear();
        result = HybridAStarResult();
    }
    virtual GraphNodeCost heuristic(HybridAStarNode* node1, HybridAStarNode* node2)
    {
        return GraphNodeCost(
            (node1->state.block(0, 0, 1, 2) - node2->state.block(0, 0, 1, 2)).norm(), 
            0
        );
    }
    virtual void getNeighbors(HybridAStarNode* node, std::vector<HybridAStarNode*>& neighbors, std::vector<GraphNodeCost>& costs)
    {
        neighbors.clear();
        costs.clear();
        for (int i = 0; i < neighborList.size(); i++)
        {
            auto & d = neighborList[i];
            Eigen::Matrix<double, 1, 3> dstate;
            dstate << d(0) * std::cos(node->state(2) + d(1)), d(0) * std::sin(node->state(2) + d(1)), d(2);
            // std::cout << dstate << std::endl;
            Eigen::Matrix<double, 1, 3> newState = node->state + dstate;
            if (newState(2) > M_PI * 2)
            {
                newState(2) = newState(2) - 2 * (M_PI);
            }
            auto neighbor = graph[node->state + dstate];
            if (neighbor != nullptr && !neighbor->occupied && neighbor->flag != IN_CLOSESET)
            {
                neighbor->state = newState;
                neighbors.push_back(neighbor);
                costs.push_back(GraphNodeCost(neighborCost[i], 0));
            }
        }
    }
    virtual bool endSearch(HybridAStarNode* node1, HybridAStarNode* node2) override
    {
        if (node1->state.block(0, 0, 1, 2).isApprox(node2->state.block(0, 0, 1, 2), 0.1))
        {
            return true;
        }
        return false;
    }
    // HybridAStarResult search(HybridAStarNode* start, HybridAStarNode* goal) override
    // {
    //     return HybridAStarResult();
    // }
private:
    // 0: 圆弧割线长度, 1: 割线与切线夹角， 2: 转角
    std::vector<Eigen::Matrix<double, 1, 3>> neighborList;
    std::vector<double> neighborCost;
};


#endif // UVE_Hybrid_ASTAR_HPP