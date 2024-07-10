#include "grid_map.hpp"

using namespace scp;

GridNode::GridNode()
{
}

GridNode::GridNode(GridNodeIndex index)
    : index(index)
{
}

GridNode::GridNode(GridNodeIndex index, Pose2D pose)
    : index(index), pose(pose)
{
}

GridNode::GridNode(const GridNode &node)
{
    this->index = node.index;
    this->pose = node.pose;
    this->search_info = node.search_info;
}

GridNode &GridNode::operator=(const GridNode &node)
{
    this->index = node.index;
    this->pose = node.pose;
    this->search_info = node.search_info;
    return *this;
}

GridMap::GridMap()
{
}

GridMap::GridMap(double minX, double minY, double maxX, double maxY, double positionResolution, int yawStep)
    : minX(minX), minY(minY), maxX(maxX), maxY(maxY), positionResolution(positionResolution), yawStep(yawStep)
{
    width = (int)((maxX - minX) / positionResolution);
    height = (int)((maxY - minY) / positionResolution);
    depth = yawStep;
    yawResolution = M_PI * 2 / yawStep;
    for (int i = 0; i < yawStep; i++)
    {
        yawList.push_back(i * yawResolution);
    }
    oriX = (int)(minX / positionResolution);
    oriY = (int)(minY / positionResolution);

    gridMap = new GridNode **[width];
    gridMapOccupied = new bool *[width];
    gridMapCollision = new std::set<int> *[width];
    for (int i = 0; i < width; i++)
    {
        gridMap[i] = new GridNode *[height];
        gridMapOccupied[i] = new bool[height];
        gridMapCollision[i] = new std::set<int>[height];
        for (int j = 0; j < height; j++)
        {
            gridMap[i][j] = new GridNode[depth];
            for (int k = 0; k < depth; k++)
            {
                gridMap[i][j][k].index.x = i;
                gridMap[i][j][k].index.y = j;
                gridMap[i][j][k].index.z = k;
                gridMap[i][j][k].pose.x = (i + oriX) * positionResolution;
                gridMap[i][j][k].pose.y = (j + oriY) * positionResolution;
                gridMap[i][j][k].pose.theta = yawList[k];
            }
            gridMapOccupied[i][j] = false;
            gridMapCollision[i][j].clear();
        }
    }
    elementOccupied.clear();
    elementCollision.clear();
}

GridMap::GridMap(const GridMap &map)
{
    if (gridMap != nullptr)
    {
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                delete[] gridMap[i][j];
            }
            delete[] gridMap[i];
            delete[] gridMapOccupied[i];
            delete[] gridMapCollision[i];
        }
        delete[] gridMap;
        delete[] gridMapOccupied;
        delete[] gridMapCollision;
    }

    this->minX = map.minX;
    this->minY = map.minY;
    this->maxX = map.maxX;
    this->maxY = map.maxY;
    this->positionResolution = map.positionResolution;
    this->yawResolution = map.yawResolution;
    this->yawStep = map.yawStep;
    this->yawList = map.yawList;
    this->width = map.width;
    this->height = map.height;
    this->depth = map.depth;
    this->oriX = map.oriX;
    this->oriY = map.oriY;

    gridMap = new GridNode **[width];
    gridMapOccupied = new bool *[width];
    gridMapCollision = new std::set<int> *[width];
    for (int i = 0; i < width; i++)
    {
        gridMap[i] = new GridNode *[height];
        gridMapOccupied[i] = new bool[height];
        gridMapCollision[i] = new std::set<int>[height];
        for (int j = 0; j < height; j++)
        {
            gridMap[i][j] = new GridNode[depth];
            for (int k = 0; k < depth; k++)
            {
                gridMap[i][j][k] = map.gridMap[i][j][k];
            }
            gridMapOccupied[i][j] = map.gridMapOccupied[i][j];
            gridMapCollision[i][j] = map.gridMapCollision[i][j];
        }
    }

    elementOccupied = map.elementOccupied;
    elementCollision = map.elementCollision;
}

GridMap::~GridMap()
{
    if (gridMap != nullptr)
    {
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                delete[] gridMap[i][j];
            }
            delete[] gridMap[i];
            delete[] gridMapOccupied[i];
            delete[] gridMapCollision[i];
        }
        delete[] gridMap;
        delete[] gridMapOccupied;
        delete[] gridMapCollision;
    }
}

void GridMap::putElement(Element &element)
{
    if (elementOccupied.count(element.id) > 0)
    {
        for (int i = 0; i < elementOccupied[element.id].size(); i++)
        {
            int x = elementOccupied[element.id][i].first;
            int y = elementOccupied[element.id][i].second;
            if (x >= 0 && x < width && y >= 0 && y < height)
                gridMapOccupied[x][y] = false;
        }
    }
    std::vector<std::pair<int,int>> occupiedList;
    element.getDispersed(occupiedList, positionResolution, oriX, oriY);
    for (int i = 0; i < occupiedList.size(); i++)
    {
        int x = occupiedList[i].first;
        int y = occupiedList[i].second;
        if (x >= 0 && x < width && y >= 0 && y < height)
            gridMapOccupied[x][y] = true;
    }
    elementOccupied[element.id] = occupiedList;

    if (elementCollision.count(element.id) > 0)
    {
        for (int i = 0; i < elementCollision[element.id].size(); i++)
        {
            int x = elementCollision[element.id][i].first;
            int y = elementCollision[element.id][i].second;
            if (x >= 0 && x < width && y >= 0 && y < height)
                gridMapCollision[x][y].erase(element.id);
        }
    }
    std::vector<std::pair<int,int>> collisionList;
    element.getCollision(collisionList, positionResolution, oriX, oriY);
    for (int i = 0; i < collisionList.size(); i++)
    {
        int x = collisionList[i].first;
        int y = collisionList[i].second;
        if (x >= 0 && x < width && y >= 0 && y < height)
            gridMapCollision[x][y].insert(element.id);
    }
}

void GridMap::removeElement(Element &element)
{
    if (elementOccupied.count(element.id) > 0)
    {
        for (int i = 0; i < elementOccupied[element.id].size(); i++)
        {
            int x = elementOccupied[element.id][i].first;
            int y = elementOccupied[element.id][i].second;
            if (x >= 0 && x < width && y >= 0 && y < height)
                gridMapOccupied[x][y] = false;
        }
        elementOccupied.erase(element.id);
    }

    if (elementCollision.count(element.id) > 0)
    {
        for (int i = 0; i < elementCollision[element.id].size(); i++)
        {
            int x = elementCollision[element.id][i].first;
            int y = elementCollision[element.id][i].second;
            if (x >= 0 && x < width && y >= 0 && y < height)
                gridMapCollision[x][y].erase(element.id);
        }
        elementCollision.erase(element.id);
    }
}

void GridMap::setOccupied(int x, int y, bool occupied)
{
}

void GridMap::setDistance(int x, int y, double distance)
{
}

GridState GridMap::operator()(GridNodeIndex &index)
{
    return (*this)(index.x, index.y, index.z);
}

GridState GridMap::operator()(int x, int y, int z)
{
    GridState state;
    if (x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth)
    {
        state.node = &gridMap[x][y][z];
        state.occupied = gridMapOccupied[x][y];
        state.collision = &gridMapCollision[x][y];
    }
    else
    {
        state.node = nullptr;
        state.occupied = false;
        state.collision = nullptr;
        RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "Z: %d", z);
    }
    return state;
}

GridState GridMap::operator()(Pose2D &pose)
{
    int x = (int)((pose.x) / positionResolution) - oriX;
    int y = (int)((pose.y) / positionResolution) - oriY;
    int z = (int)(pose.theta / yawResolution) % yawStep;
    z = z < 0 ? z + yawStep : z;
    return (*this)(x, y, z);
}

GridMap &GridMap::operator=(const GridMap &map)
{
    if (gridMap != nullptr)
    {
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                delete[] gridMap[i][j];
            }
            delete[] gridMap[i];
            delete[] gridMapOccupied[i];
            delete[] gridMapCollision[i];
        }
        delete[] gridMap;
        delete[] gridMapOccupied;
        delete[] gridMapCollision;
    }

    this->minX = map.minX;
    this->minY = map.minY;
    this->maxX = map.maxX;
    this->maxY = map.maxY;
    this->positionResolution = map.positionResolution;
    this->yawResolution = map.yawResolution;
    this->yawStep = map.yawStep;
    this->yawList = map.yawList;
    this->width = map.width;
    this->height = map.height;
    this->depth = map.depth;
    this->oriX = map.oriX;
    this->oriY = map.oriY;

    gridMap = new GridNode **[width];
    gridMapOccupied = new bool *[width];
    gridMapCollision = new std::set<int> *[width];
    for (int i = 0; i < width; i++)
    {
        gridMap[i] = new GridNode *[height];
        gridMapOccupied[i] = new bool[height];
        gridMapCollision[i] = new std::set<int>[height];
        for (int j = 0; j < height; j++)
        {
            gridMap[i][j] = new GridNode[depth];
            for (int k = 0; k < depth; k++)
            {
                gridMap[i][j][k] = map.gridMap[i][j][k];
            }
            gridMapOccupied[i][j] = map.gridMapOccupied[i][j];
            gridMapCollision[i][j] = map.gridMapCollision[i][j];
        }
    }

    elementOccupied = map.elementOccupied;
    elementCollision = map.elementCollision;

    return *this;
}

void scp::GridMap::toMsg(nav_msgs::msg::OccupancyGrid &msg)
{
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "map";
    msg.info.resolution = positionResolution;
    msg.info.width = width;
    msg.info.height = height;
    msg.info.origin.position.x = minX;
    msg.info.origin.position.y = minY;
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation.x = 0;
    msg.info.origin.orientation.y = 0;
    msg.info.origin.orientation.z = 0;
    msg.info.origin.orientation.w = 1;
    msg.data.resize(width * height);
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            msg.data[i + j * width] = gridMapOccupied[i][j] ? 100 : 0;
        }
    }
}
