#include "HybridAStar.hpp"
#include <thread>
#include <future>

HybridAStar::HybridAStar(ElementMap &map, double solveResolution, double distanceMapResolution)
{
    this->map = map;
    // distanceMap = DistanceElementMap(map, distanceMapResolution); 
    // gridMap = GridMap(map, solveResolution);
    std::future<void> distanceMapFuture = std::async(
        std::launch::async,
        genreateDistanceElementMap,
        std::ref(map),
        distanceMapResolution,
        std::ref(distanceMap)
    );
    std::future<void> gridMapFuture = std::async(
        std::launch::async,
        generateGridMap,
        std::ref(map),
        solveResolution,
        std::ref(gridMap)
    );
    distanceMapFuture.wait();
    gridMapFuture.wait();
}

HybridAStar::HybridAStar(ElementMap &map, GridMap &gridMap, DistanceElementMap &distanceMap)
{
    this->map = map;
    this->gridMap = gridMap;
    this->distanceMap = distanceMap;
}

void HybridAStar::initGridMap(ElementMap &map, double solveResolution)
{
    gridMap = GridMap(map, solveResolution);  
}

void HybridAStar::initDistanceElementMap(ElementMap &map, double distanceMapResolution)
{
    distanceMap = DistanceElementMap(map, distanceMapResolution);
}

void HybridAStar::updateAllMaps(ElementMap &map, GridMap &gridMap, DistanceElementMap &distanceMap)
{
    this->map = map;
    this->gridMap = gridMap;
    this->distanceMap = distanceMap;
    openSet.clear();
    closeSet.clear();
}

void HybridAStar::resetMap()
{
    for (auto &grid: closeSet)
    {
        grid->closed = false;
        grid->opened = false;
        grid->g = std::numeric_limits<double>::max();
        grid->h = 0;
        grid->f = std::numeric_limits<double>::max();
        grid->parent = nullptr;
        grid->it = openSet.end();
        grid->action = Action();
    }

    for (auto &grid: openSet)
    {
        grid.second->closed = false;
        grid.second->opened = false;
        grid.second->g = std::numeric_limits<double>::max();
        grid.second->h = 0;
        grid.second->f = std::numeric_limits<double>::max();
        grid.second->parent = nullptr;
        grid.second->it = openSet.end();
        grid.second->action = Action();
    }

    openSet.clear();
    closeSet.clear();
}

void HybridAStar::setIgnoreId(std::vector<int> &ignoreId)
{
    this->ignoreId = ignoreId;
}

void HybridAStar::generateKinematics()
{
    int fractionV = 1;
    int fractionW = 1;
    for (int i = 0; i < fractionV; i++)
    {
        for (int j = 0; j < fractionW; j++)
        {
            kinematics.push_back(Kinematics(task.V * (i + 1) / fractionV, task.W * (j + 1) / fractionW, task.dt));
            kinematics.push_back(Kinematics(task.V * (i + 1) / fractionV, 0, task.dt));
            kinematics.push_back(Kinematics(task.V * (i + 1) / fractionV, -task.W * (j + 1) / fractionW, task.dt));
            kinematics.push_back(Kinematics(-task.V * (i + 1) / fractionV, task.W * (j + 1) / fractionW, task.dt));
            kinematics.push_back(Kinematics(-task.V * (i + 1) / fractionV, 0, task.dt));
            kinematics.push_back(Kinematics(-task.V * (i + 1) / fractionV, -task.W * (j + 1) / fractionW, task.dt));
        }
    }
}

PlanResult HybridAStar::plan(TaskLowLevel &task)
{
    auto timeStart = std::chrono::high_resolution_clock::now();

    this->task = task;

    resetMap();

    PlanResult result;

    generateKinematics();

    Grid *start = gridMap[task.start];
    if (start == nullptr)
    {
        result.success = false;
        return result;
    }
    start->theta = task.theta0;
    Grid *goal = new Grid(task.goal, CoordI());
    goal->theta = task.theta1;

    start->g = 0;
    start->h = heuristic(start, goal);
    start->f = start->g + start->h;

    start->opened = true;
    start->it = openSet.insert(std::make_pair(start->f, start));

    std::vector<Grid *> neighbors;
    std::vector<Action> actions;
    std::vector<double> costs;
    double tentative_gcost;
    Grid* current;

    double hWeight = 1.0;

    while(!openSet.empty())
    {
        result.iterations++;

        current = openSet.begin()->second;
        
        if (heuristic(current, goal, hWeight) < task.dt * 2)
        {
            CoordD lastPosition = current->position;
            double lastTheta = current->theta;
            result.success = true;
            result.gCost = current->g;
            while (current != nullptr)
            {
                result.actions.push_back(current->action);
                current = current->parent;
            }
            std::reverse(result.actions.begin(), result.actions.end());
            auto finalAction = finalPath(
                result.actions.back().to,
                result.actions.back().theta1,
                goal->position,
                goal->theta
            );
            result.actions.insert(
                result.actions.end(),
                finalAction.begin(),
                finalAction.end());
            break;
        }

        openSet.erase(current->it);
        current->opened = false;
        current->closed = true;
        closeSet.push_back(current);

        getNeighbors(current, neighbors, actions, costs);
        for (int i = 0; i < neighbors.size(); i++)
        {
            tentative_gcost = current->g + costs[i];
            if (neighbors[i]->opened == false)
            {
                neighbors[i]->position = actions[i].to;
                neighbors[i]->theta = actions[i].theta1;
                neighbors[i]->g = tentative_gcost;
                neighbors[i]->h = heuristic(neighbors[i], goal, hWeight);
                neighbors[i]->f = neighbors[i]->g + neighbors[i]->h;
                neighbors[i]->parent = current;
                neighbors[i]->action = actions[i];
                neighbors[i]->opened = true;
                neighbors[i]->it = openSet.insert(std::make_pair(neighbors[i]->f, neighbors[i]));
            }
            else if(tentative_gcost < neighbors[i]->g)
            {
                neighbors[i]->position = actions[i].to;
                neighbors[i]->theta = actions[i].theta1;
                neighbors[i]->g = tentative_gcost;
                neighbors[i]->h = heuristic(neighbors[i], goal, hWeight);
                neighbors[i]->f = neighbors[i]->g + neighbors[i]->h;
                neighbors[i]->parent = current;
                neighbors[i]->action = actions[i];
                openSet.erase(neighbors[i]->it);
                neighbors[i]->it = openSet.insert(std::make_pair(neighbors[i]->f, neighbors[i]));
            }
        }
    }

    delete goal;

    auto timeEnd = std::chrono::high_resolution_clock::now();
    result.timeCost = (double)std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart).count() / 1000000.0;

    return result;
}

void HybridAStar::getNeighbors(Grid *current, std::vector<Grid *> &neighbors, std::vector<Action> &actions, std::vector<double> &costs)
{
    neighbors.clear();
    actions.clear();
    costs.clear();

    for (auto &k : kinematics)
    {
        CoordD position1;
        double theta1;
        k.update(current->position, current->theta, position1, theta1);
        Grid *grid = gridMap[position1];
        if (grid == nullptr || grid->closed == true) continue;
        map.agent.setGeometry(position1, theta1);
        if (checkCollision(map.agent)){
        // if (checkElementCollision(map.agent, map.elements)){
            //std::cout << "Collision" << std::endl;
            continue;
        };

        neighbors.push_back(grid);
        actions.push_back(Action(current->position, current->theta, position1, theta1, k.V, k.W, false));
        costs.push_back(k.dt);
    }
}

double HybridAStar::heuristic(Grid *current, Grid *goal, double weight)
{
    double dx = current->position.x - goal->position.x;
    double dy = current->position.y - goal->position.y;
    // double dtheta = current->theta - goal->theta;
    // if (dtheta >= 2 * M_PI)
    // {
    //     dtheta -= 2 * M_PI;
    // }
    // if (dtheta < 0)
    // {
    //     dtheta += 2 * M_PI;
    // }
    double cost = std::sqrt(dx * dx + dy * dy) / task.V * weight;// + std::abs(dtheta) / task.W;
    return cost;
}

bool HybridAStar::checkCollision(Element &element)
{
    DistanceElement* distanceElement = distanceMap[element.panning];
    if (distanceElement == nullptr) return true;
    for (auto &e : map.elements)
    {
        if (std::find(ignoreId.begin(), ignoreId.end(), e.id) != ignoreId.end()) continue;
        if (distanceElement->elementIds.count(e.id) > 0)
        {
            if (element.shape.isIntersect(e.shape))
            {
                return true;
            }
        }
    }
    return false;
}

std::vector<Action> HybridAStar::finalPath(CoordD &position0, double &theta0, CoordD &position1, double &theta1)
{
    std::vector<Action> actions;
    CoordD p0 = position0;
    double t0 = theta0;
    CoordD p1;
    double t1;
    ///TODO: check if it works
    /// step 1: turn to the goal
    double angle = std::atan2(position1.y - position0.y, position1.x - position0.x);
    double dtheta = angle - theta0;
    if (dtheta >= M_PI)
    {
        dtheta -= 2 * M_PI;
    }
    if (dtheta < -M_PI)
    {
        dtheta += 2 * M_PI;
    }
    double direction = dtheta > 0 ? 1 : -1;
    dtheta = std::abs(dtheta);
    double W = task.W * direction;
    double step = task.W * task.dt;
    while (dtheta > step)
    {
        dtheta -= step;
        t1 = t0 + W * task.dt;
        actions.push_back(Action(p0, t0, p0, t1, 0, W, false));
        t0 = t1;
    }
    actions.push_back(Action(p0, t0, p0, angle, 0, dtheta * direction / task.dt, false));
    t0 = angle;
    p1 = p0;
    /// step 2: move to the goal
    double distance = std::sqrt((position1.x - p0.x) * (position1.x - p0.x) + (position1.y - p0.y) * (position1.y - p0.y));
    step = task.V * task.dt;
    while (distance > step)
    {
        p1.x = p0.x + step * std::cos(t0);
        p1.y = p0.y + step * std::sin(t0);
        actions.push_back(Action(p0, t0, p1, t0, task.V, 0, false));
        p0 = p1;
        distance -= step;
    }
    actions.push_back(Action(p0, t0, position1, t0, distance / task.dt, 0, false));
    /// step 3: turn to the goal's direction
    dtheta = theta1 - t0;
    if (dtheta >= M_PI)
    {
        dtheta -= 2 * M_PI;
    }
    if (dtheta < -M_PI)
    {
        dtheta += 2 * M_PI;
    }
    direction = dtheta > 0 ? 1 : -1;
    dtheta = std::abs(dtheta);
    W = task.W * direction;
    step = task.W * task.dt;
    while (dtheta > step)
    {
        dtheta -= step;
        t1 = t0 + W * task.dt;
        actions.push_back(Action(p0, t0, p0, t1, 0, W, false));
        t0 = t1;
    }
    actions.push_back(Action(p0, t0, p0, theta1, 0, dtheta * direction / task.dt, false));
    return actions;
}
