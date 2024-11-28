#include "layer1_plan.hpp"

#include <chrono>

using namespace layer1;

void Layer1Plan::setExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt)
{
    vw_space.clear();
    neighbor_space.clear();
    neighbor_cost.clear();

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
            
            cv::Point3d neighbor;
            neighbor.z = w * dt;
            neighbor.y = neighbor.z / 2;
            if (j == 0)
            {
                neighbor.x = v * dt;
            }
            else
            {
                double R = v / w;
                double L = std::sqrt(2*R*R*(1 - std::cos(w * dt)));
                neighbor.x = L;
            }
            vw_space.emplace_back(cv::Point2d(v, w));
            neighbor_space.emplace_back(neighbor);

            neighbor_cost.emplace_back(std::abs(v*dt));
            if (v < 0)
            {
                neighbor_cost.back() *= 2;
            }
            else if (v == 0)
            {
                neighbor_cost.back() = dt;
            }
        }
    }
}

void Layer1Plan::bindGraph(Layer1GridGraph* g)
{
    this->graph = g;
    
}

void Layer1Plan::reset()
{
    for (auto& n : open_set)
    {
        n.second->parent = nullptr;
        n.second->flag = NOT_VISITED;
    }
    open_set.clear();
    for (auto n : close_set)
    {
        n->parent = nullptr;
        n->flag = NOT_VISITED;
    }
    close_set.clear();
    // result = Layer1PlanResult();
}

Layer1GraphNodeCost Layer1Plan::heuristic(Layer1GraphNode *node, Layer1GraphNode *goal)
{
    cv::Point2d diff = cv::Point2d(goal->state.x - node->state.x, goal->state.y - node->state.y);
    return Layer1GraphNodeCost(std::sqrt(diff.x * diff.x + diff.y * diff.y), 0);
}

void Layer1Plan::getNeighbors(Layer1GraphNode *node, std::vector<Layer1GraphNode *> &neighbors, std::vector<Layer1GraphNodeCost> &costs, std::vector<cv::Point3f> &neighbor_state, std::vector<cv::Point2d> &vws)
{
    neighbors.clear();
    costs.clear();
    neighbor_state.clear();
    vws.clear();

    for(int i = 0; i < neighbor_space.size(); i++)
    {
        auto&d = neighbor_space[i];
        cv::Point3f dstate(
            d.x*std::cos(node->state.z + d.y),
            d.x*std::sin(node->state.z + d.y),
            d.z
        );
        cv::Point3f state(
            node->state.x + dstate.x,
            node->state.y + dstate.y,
            node->state.z + dstate.z
        );
        if (state.z > M_PI)
        {
            state.z -= 2*M_PI;
        }
        else if (state.z <= -M_PI)
        {
            state.z += 2*M_PI;
        }
        auto neighbor = (*graph)(state.x, state.y, state.z);

        if ((neighbor != nullptr) && (!neighbor->collision_static) && (!neighbor->collision_dynamic) && (neighbor->flag != IN_CLOSESET))
        {
            // std::cout << "neighbor: " << state.x << "," << state.y << "," << state.z << std::endl;
            neighbors.emplace_back(neighbor);
            costs.emplace_back(Layer1GraphNodeCost(neighbor_cost[i], 0));
            neighbor_state.emplace_back(state);
            vws.emplace_back(vw_space[i]);
        }
    }
}

bool Layer1Plan::endCondition(Layer1GraphNode *node, Layer1GraphNode *goal)
{
    auto diff = goal->state - node->state;
    if (std::sqrt(diff.x*diff.x + diff.y*diff.y) < 0.1)
    {
        return true;
    }
    return false;
}

Layer1PlanResult Layer1Plan::search(Layer1GraphNode *start, Layer1GraphNode *goal)
{
    result = Layer1PlanResult();

    if (start->collision_dynamic || start->collision_static || goal->collision_dynamic || goal->collision_static)
    {
        // std::cout << "start or goal is in collision" << std::endl;
        return result;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    start->g = Layer1GraphNodeCost(0, 0);
    start->h = heuristic(start, goal);
    start->f = start->g + start->h;
    start->parent = nullptr;
    start->flag = IN_OPENSET;
    start->it = open_set.insert(std::make_pair(start->f, start));

    std::vector<Layer1GraphNode*> neighbors;
    std::vector<Layer1GraphNodeCost> costs;
    std::vector<cv::Point3f> neighbor_state;
    std::vector<cv::Point2d> vws;
    Layer1GraphNodeCost tentative_g;

    while (!open_set.empty())
    {
        result.iterations++;
        auto current = open_set.begin()->second;
        if (endCondition(current, goal))
        {
            result.success = true;
            result.cost = current->g.distance;
            while (current != nullptr)
            {
                result.path.emplace_back(current->state);
                result.vw.emplace_back(current->vw);
                current = current->parent;
            }
            std::reverse(result.path.begin(), result.path.end());
            std::reverse(result.vw.begin(), result.vw.end());
            break;
        }
        open_set.erase(current->it);
        close_set.emplace_back(current);
        current->flag = IN_CLOSESET;
        getNeighbors(current, neighbors, costs, neighbor_state, vws);
        for (int i = 0; i < neighbors.size(); i++)
        {
            auto neighbor = neighbors[i];
            auto cost = costs[i];
            auto state = neighbor_state[i];
            auto vw = vws[i];
            tentative_g = current->g + cost;
            if (neighbor->flag != IN_OPENSET)
            {
                neighbor->state = state;
                neighbor->h = heuristic(neighbor, goal);
                neighbor->g = tentative_g;
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                neighbor->vw = vw;
                neighbor->flag = IN_OPENSET;
                neighbor->it = open_set.insert(std::make_pair(neighbor->f, neighbor));
            }
            else if (tentative_g < neighbor->g)
            {
                open_set.erase(neighbor->it);
                neighbor->state = state;
                neighbor->h = heuristic(neighbor, goal);
                neighbor->g = tentative_g;
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                neighbor->vw = vw;
                neighbor->it = open_set.insert(std::make_pair(neighbor->f, neighbor));
            }
        }
    }
    reset();
    auto end_time = std::chrono::high_resolution_clock::now();
    result.planTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    return result;
}

Layer1PlanResult Layer1Plan::search(cv::Point3d start, cv::Point3d goal)
{
    auto start_node = (*graph)(start.x, start.y, start.z);
    auto goal_node = (*graph)(goal.x, goal.y, goal.z);
    if (start_node == nullptr || goal_node == nullptr)
    {
        // std::cout << "start or goal is out of the world" << std::endl;
        return Layer1PlanResult();
    }
    return search(start_node, goal_node);
}
