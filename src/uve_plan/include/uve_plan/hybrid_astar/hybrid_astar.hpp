#ifndef UVE_HYBRID_ASTAR_HPP
#define UVE_HYBRID_ASTAR_HPP

#include "uvs_tools/graph_search/graph_search.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

constexpr int GRAPH_DIMENSION = 3;
class HybridAStarNode : public GraphNodeBase<GRAPH_DIMENSION>
{
public:
    bool occupied = false;
    double v = 0, w = 0;
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
class HybridAStarResult : public GraphSearchResultBase<HybridAStarTraceType>
{
public:
    std::vector<Eigen::Matrix<double, 1, 2>> vw;
};
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
                if (i == 0 && j == 0)
                {
                    continue;
                }
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
                vwList.push_back(Eigen::Matrix<double, 1, 2>(v, w));
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
    void getNeighbors(HybridAStarNode* node, 
        std::vector<HybridAStarNode*>& neighbors, 
        std::vector<GraphNodeCost>& costs)
        {}
    void getNeighbors(HybridAStarNode* node, 
        std::vector<HybridAStarNode*>& neighbors, 
        std::vector<GraphNodeCost>& costs,
        std::vector<Eigen::Matrix<double, 1, 3>>& neighborState,
        std::vector<Eigen::Matrix<double, 1, 2>>& neighborVW)
    {
        neighbors.clear();
        costs.clear();
        neighborState.clear();
        neighborVW.clear();
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
            auto neighbor = graph[newState];
            if (neighbor != nullptr && !neighbor->occupied && neighbor->flag != IN_CLOSESET)
            {
                neighborState.push_back(newState);
                neighborVW.push_back(vwList[i]);
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
    HybridAStarResult search(HybridAStarNode* start, HybridAStarNode* goal) override
    {
        reset();
        auto start_time = std::chrono::high_resolution_clock::now();
        
        start->g = GraphNodeCost(0, 0);
        start->h = heuristic(start, goal);
        start->f = start->g + start->h;
        start->flag = IN_OPENSET;
        start->it = openSet.insert(std::make_pair(start->f, start));
        while (!openSet.empty())
        {
            result.iterations++;
            auto current = (HybridAStarNode*)openSet.begin()->second;
            // std::cout << "current: " << current->index << std::endl;

            if (endSearch(current, goal))
            {
                result.success = true;
                result.cost = current->g.distance;
                while (current != nullptr)
                {
                    result.trace.push_back(current->state);
                    result.vw.push_back(Eigen::Matrix<double, 1, 2>(current->v, current->w));
                    current = (HybridAStarNode*)current->parent;
                }
                std::reverse(result.trace.begin(), result.trace.end());
                std::reverse(result.vw.begin(), result.vw.end());
                break;
            }
            openSet.erase(current->it);
            closeSet.push_back(current);
            current->flag = IN_CLOSESET;

            std::vector<HybridAStarNode*> neighbors;
            std::vector<GraphNodeCost> costs;
            std::vector<Eigen::Matrix<double, 1, 3>> neighborState;
            std::vector<Eigen::Matrix<double, 1, 2>> neighborVW;
            // RCLCPP_INFO(rclcpp::get_logger("test"), "current: %f,%f,%f", current->state(0), current->state(1), current->state(2));
            getNeighbors(current, neighbors, costs, neighborState, neighborVW);
            // RCLCPP_INFO(rclcpp::get_logger("test"), "neighbors: %d", neighbors.size());
            for (int i = 0; i < neighbors.size(); i++)
            {
                auto neighbor = neighbors[i];
                // if (neighbor->flag == IN_CLOSESET)
                // {
                //     continue;
                // }
                GraphNodeCost tentative_g = current->g + costs[i];
                if (neighbor->flag != IN_OPENSET)
                {
                    neighbor->h = heuristic(neighbor, goal);
                    neighbor->g = tentative_g;
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;
                    neighbor->flag = IN_OPENSET;
                    neighbor->state = neighborState[i];
                    neighbor->v = neighborVW[i](0);
                    neighbor->w = neighborVW[i](1);
                    neighbor->it = openSet.insert(std::make_pair(neighbor->f, neighbor));
                }
                else if (tentative_g < neighbor->g)
                {
                    openSet.erase(neighbor->it);
                    neighbor->h = heuristic(neighbor, goal);
                    neighbor->g = tentative_g;
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;
                    neighbor->state = neighborState[i];
                    neighbor->v = neighborVW[i](0);
                    neighbor->w = neighborVW[i](1);
                    neighbor->it = openSet.insert(std::make_pair(neighbor->f, neighbor));
                }
            }
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        result.planTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        return result;
    }
private:
    std::vector<Eigen::Matrix<double, 1, 2>> vwList;
    // 0: 圆弧割线长度, 1: 割线与切线夹角， 2: 转角
    std::vector<Eigen::Matrix<double, 1, 3>> neighborList;
    std::vector<double> neighborCost;
};


#endif // UVE_Hybrid_ASTAR_HPP