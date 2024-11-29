#include "layer3_plan.hpp"

#include <thread>
#include <future>

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

    if (node->domainLabel == goalDomain)
    {
        return;
    }

    std::vector<Layer3Plan::threadin> ins;

    std::vector<std::future<Layer3Plan::threadout>> futures;

    auto possible_edges = map->domain_edges[node->domainLabel];
    // std::cout << "possible_edges: " << possible_edges.size() << std::endl;
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
            // std::cout << "goalDomain" << std::endl;
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
            Layer3Plan::threadin  in = {
                .Astart = Astart,
                .Cname = Cname,
                .Cgoal = Cgoal,
                .Agoal = Agoal,
                .dynamic_state = temp_dynamic_state,
                .node = node,
                .edge_id = edge_id,
                .checkPoint_id = i
            };

            // std::cout << "----------------" << std::endl;
            // std::cout << "Astart: " << Astart.x << " " << Astart.y << " " << Astart.z << std::endl;
            // std::cout << "Cname: " << Cname << std::endl;
            // std::cout << "Cgoal: " << Cgoal.x << " " << Cgoal.y << " " << Cgoal.z << std::endl;
            // std::cout << "Agoal: " << Agoal.x << " " << Agoal.y << " " << Agoal.z << std::endl;
            // std::cout << "cur domain: " << node->domainLabel << std::endl;
            // std::cout << "edge_id: " << edge_id << std::endl;
            // std::cout << "checkPoint_id: " << i << std::endl;
            // std::cout << "----------------" << std::endl;

            ins.push_back(in);
            // planner->updateGraph(temp_dynamic_state);
            // // std::cout << "searching " << Cname << " " << i << std::endl;
            // auto ret = planner->search(Astart, Cname, Cgoal, Agoal);
            // if (ret.success)
            // {
            //     // update the dynamic state
            //     auto tempIt = std::find_if(temp_dynamic_state.list.begin(), temp_dynamic_state.list.end(), [&](const uve_message::msg::UveDynamicStatus& d){
            //         return d.name == Cname;
            //     });
            //     if (tempIt != temp_dynamic_state.list.end())
            //     {
            //         tempIt->pose.x = Cgoal.x;
            //         tempIt->pose.y = Cgoal.y;
            //         tempIt->pose.theta = Cgoal.z;
            //     }
            //     neighbors.push_back((*graph)(edge_id, i));
            //     costs.push_back(ret.cost);
            //     neighbor_domain.push_back(edge.otherDomain(node->domainLabel));
            //     neighbor_agent_state.push_back(ret.path_m.back());
            //     neighbor_dynamic_state.push_back(temp_dynamic_state);
            //     neighbor_path_from_parent.push_back(ret);
            //     neighbor_via_edge.push_back(edge_id);
            //     neighbor_index.push_back(std::make_pair(edge_id, i));
            //     // std::cout << "success cost:"  << ret.cost << std::endl;
            // }
        }
    }

    for (int i = 0; i < ins.size(); i++)
    {
        futures.push_back(std::async(std::launch::async, &Layer3Plan::searchThread, this, i, std::ref(ins[i])));
    }
    for (int i = 0; i < futures.size(); i++)
    {
        auto ret = futures[i].get();
        if (ret.neighbor != nullptr)
        {
            neighbors.push_back(ret.neighbor);
            costs.push_back(ret.cost);
            neighbor_domain.push_back(ret.domain);
            neighbor_agent_state.push_back(ret.agent_state);
            neighbor_dynamic_state.push_back(ret.dynamic_state);
            neighbor_path_from_parent.push_back(ret.path_from_parent);
            neighbor_via_edge.push_back(ret.via_edge);
            neighbor_index.push_back(ret.index);
        }
    }
}

Layer3Plan::threadout Layer3Plan::searchThread(int id, Layer3Plan::threadin &in)
{
    Layer3Plan::threadout out;
    planner[id].updateGraph(in.dynamic_state);
    auto ret = planner[id].search(in.Astart, in.Cname, in.Cgoal, in.Agoal);
    if (ret.success)
    {
        // update the dynamic state
        auto tempIt = std::find_if(in.dynamic_state.list.begin(), in.dynamic_state.list.end(), [&](const uve_message::msg::UveDynamicStatus& d){
            return d.name == in.Cname;
        });
        if (tempIt != in.dynamic_state.list.end())
        {
            tempIt->pose.x = in.Cgoal.x;
            tempIt->pose.y = in.Cgoal.y;
            tempIt->pose.theta = in.Cgoal.z;
        }
        out.neighbor = (*graph)(in.edge_id, in.checkPoint_id);
        out.cost = ret.cost;
        out.domain = map->edges[in.edge_id].otherDomain(in.node->domainLabel);
        if (ret.path_a.size() > 0)
        {
            out.agent_state = ret.path_a.back();
        }
        else if (ret.path_c.size() > 0)
        {
            out.agent_state = ret.path_c.back();
        }
        else
        {
            out.agent_state = ret.path_a.back();
        }
        // out.agent_state = ret.path_m.back();
        out.dynamic_state = in.dynamic_state;
        out.path_from_parent = ret;
        out.via_edge = in.edge_id;
        out.index = std::make_pair(in.edge_id, in.checkPoint_id);
        // neighbors.push_back((*graph)(edge_id, i));
        // costs.push_back(ret.cost);
        // neighbor_domain.push_back(edge.otherDomain(node->domainLabel));
        // neighbor_agent_state.push_back(ret.path_m.back());
        // neighbor_dynamic_state.push_back(temp_dynamic_state);
        // neighbor_path_from_parent.push_back(ret);
        // neighbor_via_edge.push_back(edge_id);
        // neighbor_index.push_back(std::make_pair(edge_id, i));
        // std::cout << "success cost:"  << ret.cost << std::endl;
    }
    else
    {
        out.neighbor = nullptr;
    }
    return out;
}

Layer3PlanResult Layer3Plan::search(cv::Point3d start, cv::Point3d goal)
{
    Layer3PlanResult result;

    auto time_start = std::chrono::high_resolution_clock::now();

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
    startNode.h = heuristic(&startNode, goal);
    startNode.f = startNode.g + startNode.h;
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
            break;
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
                // neighbor->via_edge.push_back(via_edge);
                neighbor->via_edge = current->via_edge;
                neighbor->via_edge.push_back(via_edge);
                neighbor->agent_state = agent_state;
                neighbor->dynamic_state = dynamic_state;
                neighbor->path_from_parent = path_from_parent;
                neighbor->parent = current;
                neighbor->g = current->g + cost;
                neighbor->h = heuristic(neighbor, goal);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->flag = Layer3GraphNodeFlag::IN_OPENSET;
                neighbor->it = open_set.insert(std::make_pair(neighbor->f, neighbor));
            }
            else if (current->g + cost < neighbor->g)
            {
                open_set.erase(neighbor->it);
                neighbor->domainLabel = domain;
                neighbor->index = index;
                // neighbor->via_edge.push_back(via_edge);
                neighbor->via_edge = current->via_edge;
                neighbor->via_edge.push_back(via_edge);
                neighbor->agent_state = agent_state;
                neighbor->dynamic_state = dynamic_state;
                neighbor->path_from_parent = path_from_parent;
                neighbor->parent = current;
                neighbor->g = current->g + cost;
                neighbor->h = heuristic(neighbor, goal);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->it = open_set.insert(std::make_pair(neighbor->f, neighbor));
            }
        }
    }
    open_set.clear();

    auto time_end = std::chrono::high_resolution_clock::now();
    result.planTime = std::chrono::duration<double, std::nano>(time_end - time_start).count();
    return result;
}

double Layer3Plan::heuristic(Layer3SearchNode *node, cv::Point3d goal)
{
    auto diff = goal - node->agent_state;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y);
}
