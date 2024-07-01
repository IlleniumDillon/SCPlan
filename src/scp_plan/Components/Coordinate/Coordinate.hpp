#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Eigen>

class CoordSystem;
extern CoordSystem globalCoordSystem;

/// @brief a class to represent a coordinate in 3D space, also can be used in 2D space
/// @tparam T data type of the coordinate
template <typename T>
class Coord
{
public:
    /// @brief value of x, y, z
    T x, y, z;
    /// @brief the coordinate system of the coordinate, default is the global coordinate system.
    ///        if it`s nullptr, it is a bad coordinate
    /// @see globalCoordSystem
    CoordSystem *coordSystem;
public:
    /// @brief constructor of new coordinate
    /// @param x coordinate value x
    /// @param y coordinate value y
    /// @param z coordinate value z
    /// @param coordSystem the coordinate system of the coordinate, default is the global coordinate system
    Coord(T x, T y, T z = 0, CoordSystem *coordSystem = &globalCoordSystem)
        : x(x), y(y), z(z), coordSystem(coordSystem) {}
    /// @brief copy constructor of new coordinate
    /// @param coord base coordinate
    Coord(const Coord<T> &coord)
        : x(coord.x), y(coord.y), z(coord.z), coordSystem(coord.coordSystem) {}
    /// @brief default constructor of new coordinate
    Coord()
        : x(0), y(0), z(0), coordSystem(&globalCoordSystem) {}
    
    Coord<T> operator+(const Coord<T> &coord) const
    {
        return Coord<T>(x + coord.x, y + coord.y, z + coord.z);
    }
    Coord<T> operator-(const Coord<T> &coord) const
    {
        return Coord<T>(x - coord.x, y - coord.y, z - coord.z);
    }
    Coord<T> operator*(const T &scalar) const
    {
        return Coord<T>(x * scalar, y * scalar, z * scalar);
    }
    Coord<T> operator/(const T &scalar) const
    {
        return Coord<T>(x / scalar, y / scalar, z / scalar);
    }
    Coord<T> operator-() const
    {
        return Coord<T>(-x, -y, -z);
    }
    Coord<T> &operator+=(const Coord<T> &coord)
    {
        x += coord.x;
        y += coord.y;
        z += coord.z;
        return *this;
    }
    Coord<T> &operator-=(const Coord<T> &coord)
    {
        x -= coord.x;
        y -= coord.y;
        z -= coord.z;
        return *this;
    }
    Coord<T> &operator*=(const T &scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
    Coord<T> &operator/=(const T &scalar)
    {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }
    bool operator==(const Coord<T> &coord) const
    {
        return x == coord.x && y == coord.y && z == coord.z;
    }
    bool operator!=(const Coord<T> &coord) const
    {
        return x != coord.x || y != coord.y || z != coord.z;
    }
    /// @brief 3D dot product
    /// @param coord other coordinate
    /// @return value of dot product
    double dot(const Coord<T> &coord) const
    {
        return x * coord.x + y * coord.y + z * coord.z;
    }
    /// @brief 3D cross product
    /// @param coord other coordinate
    /// @return value of cross product
    Coord<T> cross(const Coord<T> &coord) const
    {
        return Coord<T>(y * coord.z - z * coord.y,
                        z * coord.x - x * coord.z,
                        x * coord.y - y * coord.x);
    }
    /// @brief 2D cross product, consider x, y only
    /// @param coord other coordinate
    /// @return value of cross product
    double cross2(const Coord<T> &coord) const
    {
        return x * coord.y - y * coord.x;
    }
    double norm() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    double norm2() const
    {
        return x * x + y * y + z * z;
    }
    Coord<T> normalized() const
    {
        return *this / norm();
    }
    Coord<T> &normalize()
    {
        return *this /= norm();
    }
    /// @brief coordinate transformation
    /// @param targetCoordSystem new coordinate system
    /// @return the coordinate in the new coordinate system
    Coord<T> transform(CoordSystem *targetCoordSystem) const;
};

typedef Coord<double> CoordD;
typedef Coord<float> CoordF;
typedef Coord<int> CoordI;

/*
    all coord systems are defined in the global coordinate system
    the global coordinate system is the default coordinate system
        y
        ^
        |
       o------> x
       up is z
    (global coordinate system)
*/
/// @brief a class to represent a coordinate system
class CoordSystem
{
public:
    /// @brief unique id of the coordinate system
    int id;
public:
    /// @brief default constructor of new coordinate system. this will create global coordinate system.
    /// @note in theorie, there should be only one global coordinate system, and it is called automatically.
    ///       if called more than once, it will print an error message.
    CoordSystem();
    /// @brief constructor of new coordinate system
    /// @param transform the transformation matrix from the global coordinate system to the new coordinate system
    /// @note a matrix of transform matrix is stored as a global variable, and can be used to transform coordinates
    CoordSystem(Eigen::Matrix4d transform);
    /// @brief get the inverse of a transformation matrix
    /// @param transform a transformation matrix
    /// @return the inverse of the transformation matrix
    Eigen::Matrix4d invT(Eigen::Matrix4d transform);
};

template <typename T>
inline Coord<T> Coord<T>::transform(CoordSystem *targetCoordSystem) const
{
    /*
        P' = T * P
    */
    extern std::vector<std::vector<Eigen::Matrix4d>> coordTransforms; 
    Eigen::Matrix4d transform = coordTransforms.at(coordSystem->id).at(targetCoordSystem->id);
    Eigen::Vector4d coord(x, y, z, 1);
    Eigen::Vector4d transformedCoord = transform * coord;
    return Coord<T>(transformedCoord(0), transformedCoord(1), transformedCoord(2), targetCoordSystem);
}

#endif // COORDINATE_HPP
