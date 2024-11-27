#include "layer3_plan.hpp"

using namespace layer3;
using namespace layer2;
using namespace layer1;

void Layer3Plan::bindGraph(Layer3SearchGraph *graph)
{
    this->graph = graph;
}

void Layer3Plan::bindPixMap(Layer3PixMap *map)
{
    this->map = map;
}

void Layer3Plan::bindPlanner(layer2::Layer2Plan *planner)
{
    this->planner = planner;
}

void Layer3Plan::getNeighbors(Layer3SearchNode *node,
                                std::vector<Layer3SearchNode *> &neighbors,
                                std::vector<double> &costs,
                                std::vector<int> &neighbor_domain,
                                std::vector<cv::Point3f> &neighbor_agent_state,
                                std::vector<uve_message::msg::UveDynamicStatusList> &neighbor_dynamic_state,
                                std::vector<Layer2PlanResult> &neighbor_path_from_parent,
                                std::vector<int> &neighbor_via_edge,
                                std::vector<std::pair<int, int>> &neighbor_index)
{
    neighbors.clear();
    costs.clear();
    neighbor_domain.clear();
    neighbor_agent_state.clear();
    neighbor_dynamic_state.clear();
    neighbor_path_from_parent.clear();
    neighbor_via_edge.clear();
    neighbor_index.clear();

    auto possible_edges = map->domain_edges[node->domainLabel];
    std::cout << "possible_edges: " << possible_edges.size() << std::endl;
    for (auto edge_id : possible_edges)
    {
        if (std::find(node->via_edge.begin(), node->via_edge.end(), edge_id) != node->via_edge.end())
        {
            continue;
        }
        auto edge = map->edges[edge_id];    
        
        cv::Point3d Astart = node->agent_state;
        cv::Point3d Agoal (std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        if (edge.otherDomain(node->domainLabel) == goalDomain)
        {
            std::cout << "goalDomain" << std::endl;
            Agoal = goal;
        }
        std::string Cname = edge.name;
        cv::Point3d Cgoal;
        auto CstartIt = std::find_if(node->dynamic_state.list.begin(), node->dynamic_state.list.end(), [&](const uve_message::msg::UveDynamicStatus& d){
            return d.name == Cname;
        });
        if (CstartIt == node->dynamic_state.list.end())
        {
            continue;
        }
        
        cv::Point3d Cstart = cv::Point3d(CstartIt->pose.x, CstartIt->pose.y, CstartIt->pose.theta);
        for (int i = 0; i < graph->checkPoint_num; i++)
        {
            auto temp_dynamic_state = node->dynamic_state;
            if ((*graph)(edge_id, i)->flag == Layer3GraphNodeFlag::IN_CLOSESET)
            {
                continue;
            }
            Cgoal = cv::Point3d(
                graph->checkPoints[i].x * std::cos(edge.direction) + Cstart.x,
                graph->checkPoints[i].x * std::sin(edge.direction) + Cstart.y,
                graph->checkPoints[i].y + Cstart.z
            );
            planner->updateGraph(temp_dynamic_state);
            std::cout << "searching " << Cname << " " << i << std::endl;
            auto ret = planner->search(Astart, Cname, Cgoal, Agoal);
            if (ret.success)
            {
                // update the dynamic state
                auto tempIt = std::find_if(temp_dynamic_state.list.begin(), temp_dynamic_state.list.end(), [&](const uve_message::msg::UveDynamicStatus& d){
                    return d.name == Cname;
                });
                if (tempIt != temp_dynamic_state.list.end())
                {
                    tempIt->pose.x = Cgoal.x;
                    tempIt->pose.y = Cgoal.y;
                    tempIt->pose.theta = Cgoal.z;
                }
                neighbors.push_back((*graph)(edge_id, i));
                costs.push_back(ret.cost);
                neighbor_domain.push_back(edge.otherDomain(node->domainLabel));
                neighbor_agent_state.push_back(ret.path_m.back());
                neighbor_dynamic_state.push_back(temp_dynamic_state);
                neighbor_path_from_parent.push_back(ret);
                neighbor_via_edge.push_back(edge_id);
                neighbor_index.push_back(std::make_pair(edge_id, i));
                std::cout << "success cost:"  << ret.cost << std::endl;
            }
        }
    }
}

Layer3PlanResult Layer3Plan::search(cv::Point3d start, cv::Point3d goal)
{
    Layer3PlanResult result;
    auto pstartDomain = (*map)(start.x, start.y);
    auto pgoalDomain = (*map)(goal.x, goal.y);
    // 超界
    if (pstartDomain == nullptr || pgoalDomain == nullptr)
    {
        return result;
    }
    // 碰撞
    if (*pstartDomain < 0 || *pgoalDomain < 0)
    {
        return result;
    }
    //如果起点和终点在同一个区域内，退化成layer2的退化搜索
    if (*pstartDomain == *pgoalDomain)
    {
        auto ret = planner->search(start, "", cv::Point3d(0, 0, 0), goal);
        if (ret.success)
        {
            result.success = true;
            result.cost = ret.cost;
            result.path.push_back(ret);
        }
        return result;
    }

    // 初始化搜索
    Layer3SearchNode startNode;
    startNode.domainLabel = *pstartDomain;
    startNode.agent_state = start;
    startNode.dynamic_state = planner->dynamic_state;
    startNode.g = 0;
    startNode.h = 0;
    startNode.f = 0;
    startNode.flag = Layer3GraphNodeFlag::IN_OPENSET;
    startNode.it = open_set.insert(std::make_pair(0, &startNode));

    goalDomain = *pgoalDomain;
    this->goal = goal;

    while (!open_set.empty())
    {
        auto current = open_set.begin()->second;

        if (current->domainLabel == *pgoalDomain)
        {
            result.success = true;
            result.cost = current->g;
            while (current->parent != nullptr)
            {
                result.path.push_back(current->path_from_parent);
                current = current->parent;
            }
            std::reverse(result.path.begin(), result.path.end());
        }

        open_set.erase(open_set.begin());
        current->flag = Layer3GraphNodeFlag::IN_CLOSESET;

        std::vector<Layer3SearchNode*> neighbors;
        std::vector<double> costs;
        std::vector<int> neighbor_domain;
        std::vector<cv::Point3f> neighbor_agent_state;
        std::vector<uve_message::msg::UveDynamicStatusList> neighbor_dynamic_state;
        std::vector<Layer2PlanResult> neighbor_path_from_parent;
        std::vector<int> neighbor_via_edge;
        std::vector<std::pair<int, int>> neighbor_index;

        getNeighbors(current,
                        neighbors,
                        costs,
                        neighbor_domain,
                        neighbor_agent_state,
                        neighbor_dynamic_state,
                        neighbor_path_from_parent,
                        neighbor_via_edge,
                        neighbor_index);

        for (int i = 0; i < neighbors.size(); i++)
        {
            auto neighbor = neighbors[i];
            auto cost = costs[i];
            auto domain = neighbor_domain[i];
            auto agent_state = neighbor_agent_state[i];
            auto dynamic_state = neighbor_dynamic_state[i];
            auto path_from_parent = neighbor_path_from_parent[i];
            auto via_edge = neighbor_via_edge[i];
            auto index = neighbor_index[i];

            if (neighbor->flag == Layer3GraphNodeFlag::NOT_VISITED)
            {
                neighbor->domainLabel = domain;
                neighbor->index = index;
                neighbor->via_edge.push_back(via_edge);
                neighbor->agent_state = agent_state;
                neighbor->dynamic_state = dynamic_state;
                neighbor->path_from_parent = path_from_parent;
                neighbor->parent = current;
                neighbor->g = current->g + cost;
                neighbor->h = 0;
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->flag = Layer3GraphNodeFlag::IN_OPENSET;
                neighbor->it = open_set.insert(std::make_pair(neighbor->f, neighbor));
            }
            else if (current->g + cost < neighbor->g)
            {
                open_set.erase(neighbor->it);
                neighbor->domainLabel = domain;
                neighbor->index = index;
                neighbor->via_edge.push_back(via_edge);
                neighbor->agent_state = agent_state;
                neighbor->dynamic_state = dynamic_state;
                neighbor->path_from_parent = path_from_parent;
                neighbor->parent = current;
                neighbor->g = current->g + cost;
                neighbor->h = 0;
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->it = open_set.insert(std::make_pair(neighbor->f, neighbor));
            }
        }
    }
    open_set.clear();
    return result;
}
