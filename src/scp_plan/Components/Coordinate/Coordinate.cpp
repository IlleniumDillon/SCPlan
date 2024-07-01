#include "Coordinate.hpp"

/// @brief a class generate unique id code
/// @note this class is working in single instance mode
class _uniqueIdGenerator
{
public:
    /// @brief the next id to be generated
    int nextId = 0;
    /// @brief the list of used id
    std::vector<int> usedIds;
public:
    /// @brief get the instance of the class
    static _uniqueIdGenerator &getInstance()
    {
        static _uniqueIdGenerator instance;
        return instance;
    }
    /// @brief generate a new id
    int generateId()
    {
        int id = nextId;
        nextId++;
        usedIds.push_back(id);
        return id;
    }
};

/// @brief all coord systems are defined in the global coordinate system
std::vector<CoordSystem *> coordSystems;

/// @brief a matrix of transform matrix is stored as a global variable, and can be used to transform coordinates
///     \
/// from \ to   1   2   3   4
///       \
///     1
///     2
///     3
///     4
std::vector<std::vector<Eigen::Matrix4d>> coordTransforms;

/// @brief the global coordinate system
CoordSystem globalCoordSystem = CoordSystem();

CoordSystem::CoordSystem()
{
    if (coordSystems.size() != 0)
    {
        std::cerr << "Only one global coordinate system is allowed" << std::endl;
        return;
    }
    id = _uniqueIdGenerator::getInstance().generateId();
    coordSystems.push_back(this);
    std::vector<Eigen::Matrix4d> temp;
    temp.push_back(Eigen::Matrix4d::Identity());
    coordTransforms.push_back(temp);
}

CoordSystem::CoordSystem(Eigen::Matrix4d transform)
{
    id = _uniqueIdGenerator::getInstance().generateId();
    coordSystems.push_back(this);
    for (int i = 0; i < coordSystems.size()-1; i++)
    {
        Eigen::Matrix4d temp = invT(transform) * coordTransforms.at(i).at(0);//invT(coordTransforms.at(i).at(0)) * transform;
        coordTransforms.at(i).push_back(temp);
    }
    std::vector<Eigen::Matrix4d> newLine;
    for (int i = 0; i < coordSystems.size()-1; i++)
    {
        Eigen::Matrix4d temp = invT(coordTransforms.at(i).back());
        newLine.push_back(temp);
    }
    newLine.push_back(Eigen::Matrix4d::Identity());
    coordTransforms.push_back(newLine);
}

Eigen::Matrix4d CoordSystem::invT(Eigen::Matrix4d transform)
{
    double nx = transform(0, 0);
    double ny = transform(1, 0);
    double nz = transform(2, 0);
    double ox = transform(0, 1);
    double oy = transform(1, 1);
    double oz = transform(2, 1);
    double ax = transform(0, 2);
    double ay = transform(1, 2);
    double az = transform(2, 2);
    double px = transform(0, 3);
    double py = transform(1, 3);
    double pz = transform(2, 3);
    Eigen::Vector3d n(nx, ny, nz);
    Eigen::Vector3d o(ox, oy, oz);
    Eigen::Vector3d a(ax, ay, az);
    Eigen::Vector3d p(px, py, pz);
    Eigen::Matrix4d invT;
    invT << nx, ny, nz, -n.dot(p),
            ox, oy, oz, -o.dot(p),
            ax, ay, az, -a.dot(p),
            0, 0, 0, 1;
    return invT;
}
