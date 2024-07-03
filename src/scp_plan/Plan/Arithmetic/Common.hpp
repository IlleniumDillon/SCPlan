#ifndef COMMON_HPP
#define COMMON_HPP

#include "Shape.hpp"
#include "Map.hpp"

class TaskLowLevel
{
public:
    CoordD start;
    double theta0;
    CoordD goal;
    double theta1;
    double V, W;
    double dt;
    //double resolution;
public:
    TaskLowLevel(CoordD start, double theta0, CoordD goal, double theta1, double V, double W, double dt)
        : start(start), theta0(theta0), goal(goal), theta1(theta1), V(V), W(W), dt(dt) {}
    TaskLowLevel(){}
};

class TaskCarry
{
    ///TODO:
public:
    CoordD agentStart;
    double agentTheta0;

    int elementId;
    CoordD elementTargetPosition;
    double elementTargetTheta;

    CoordD agentGoal;
    double agentTheta1;

    double V, W;
    double dt;

public:
    TaskCarry(CoordD agentStart, double agentTheta0, int elementId, CoordD elementTargetPosition, double elementTargetTheta, CoordD agentGoal, double agentTheta1, double V, double W, double dt)
        : agentStart(agentStart), agentTheta0(agentTheta0), elementId(elementId), elementTargetPosition(elementTargetPosition), elementTargetTheta(elementTargetTheta), agentGoal(agentGoal), agentTheta1(agentTheta1), V(V), W(W), dt(dt) {}
    TaskCarry(){}
};

class Action
{
public:
    CoordD from;
    double theta0;
    CoordD to;
    double theta1;
    double V, W;
    bool lift;
    bool _final;
public:
    Action(CoordD from, double theta0, CoordD to, double theta1, double V, double W, bool lift, bool _final = false)
        : from(from), theta0(theta0), to(to), theta1(theta1), V(V), W(W), lift(lift),_final(_final) {}
    Action(){}
};

class PlanResult
{
public:
    bool success;
    int iterations;
    double timeCost;
    double gCost;
    std::vector<Action> actions;
public:
    PlanResult(bool success, int iterations, double timeCost, double gCost, std::vector<Action> actions)
        : success(success), iterations(iterations), timeCost(timeCost), gCost(gCost), actions(actions) {}
    PlanResult()
        : success(false), iterations(0), timeCost(0), gCost(0) {}
};

class GridInfo
{
public:
    CoordD position;
    double theta;
public:
    GridInfo(CoordD position, double theta)
    {
        this->position = position;
        this->theta = theta;
    }
    GridInfo()
    {
        this->position = CoordD();
        this->theta = 0;
    }
};

class Grid
{
public:
    CoordD position;
    CoordI index;
    double theta;

    double g;
    double h;
    double f;

    Grid *parent;

    bool closed;
    bool opened;

    Action action;

    std::multimap<double, Grid*>::iterator it;
public:
    Grid(CoordD position, CoordI index)
    {
        this->position = position;
        this->index = index;
        this->theta = 0;
        this->g = std::numeric_limits<double>::max();
        this->h = 0;
        this->f = std::numeric_limits<double>::max();
        this->parent = nullptr;
        this->closed = false;
        this->opened = false;
    }
    Grid()
    {
        this->position = CoordD();
        this->index = CoordI();
        this->theta = 0;
        this->g = std::numeric_limits<double>::max();
        this->h = 0;
        this->f = std::numeric_limits<double>::max();
        this->parent = nullptr;
        this->closed = false;
        this->opened = false;
    }
};

class GridMap
{
public:
    Grid** grids;
    CoordI size;
    double resolution;
    double minX, minY, maxX, maxY;
public:
    GridMap()
    {
        this->grids = nullptr;
        this->size = CoordI();
        this->resolution = 0;
        this->minX = 0;
        this->minY = 0;
        this->maxX = 0;
        this->maxY = 0;
    }
    GridMap(ElementMap &map, double &resolution)
    {
        this->minX = map.minX;
        this->minY = map.minY;
        this->maxX = map.maxX;
        this->maxY = map.maxY;
        this->resolution = resolution;

        double width = map.maxX - map.minX;
        double height = map.maxY - map.minY;
        this->size = CoordI(width / resolution, height / resolution);

        this->grids = new Grid*[this->size.x];
        for (int i = 0; i < this->size.x; i++)
        {
            this->grids[i] = new Grid[this->size.y];
            for (int j = 0; j < this->size.y; j++)
            {
                CoordD position = CoordD(map.minX + i * resolution, map.minY + j * resolution);
                CoordI index = CoordI(i, j);
                this->grids[i][j] = Grid(position, index);
            }
        }
    }
    ~GridMap()
    {
        if (this->grids == nullptr)
        {
            return;
        }
        for (int i = 0; i < this->size.x; i++)
        {
            delete[] this->grids[i];
        }
        delete[] this->grids;
        this->grids = nullptr;
    }

    GridMap& operator=(const GridMap &gridMap)
    {
        if (this->grids != nullptr)
        {
            for (int i = 0; i < this->size.x; i++)
            {
                delete[] this->grids[i];
            }
            delete[] this->grids;
        }
        this->size = gridMap.size;
        this->resolution = gridMap.resolution;
        this->minX = gridMap.minX;
        this->minY = gridMap.minY;
        this->maxX = gridMap.maxX;
        this->maxY = gridMap.maxY;
        this->grids = new Grid*[this->size.x];
        for (int i = 0; i < this->size.x; i++)
        {
            this->grids[i] = new Grid[this->size.y];
            for (int j = 0; j < this->size.y; j++)
            {
                this->grids[i][j] = gridMap.grids[i][j];
            }
        }
        return *this;
    }   

    Grid* operator[](CoordI index)
    {
        return (*this)(index.x, index.y);
    }
    Grid* operator[](CoordD position)
    {
        return (*this)(position.x, position.y);
    }
    Grid* operator()(int x, int y)
    {
        if (x < 0 || x >= this->size.x || y < 0 || y >= this->size.y)
        {
            return nullptr;
        }
        return &this->grids[x][y];
    }
    Grid* operator()(double x, double y)
    {
        int i = (x - this->minX) / this->resolution;
        int j = (y - this->minY) / this->resolution;
        return (*this)(i, j);
    }

};


void generateGridMap(ElementMap &map, double resolution, GridMap &gridMap);

#endif // COMMON_HPP