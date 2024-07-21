#ifndef PIX_MAP_HPP
#define PIX_MAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <bitset>

#include <opencv2/opencv.hpp>

#include "grid_map.hpp"

#define PIX_FREE (-1)
#define PIX_OCCUPIED (-2)
#define PIX_TEMP (-3)
#define PIX_OPENING (-4)

namespace scp
{

class PixMap
{
public:
    PixMap();
    PixMap(GridMap& grid_map);

    PixMap(const PixMap& pix_map);
    PixMap& operator=(const PixMap& pix_map);

    ~PixMap();
public:
    int width;
    int height;
    int domain_num;
    double resolution;
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    int **pix_map;
    std::vector<Point> domain_center;
public:
    void convertGridMap(GridMap& grid_map);
    int* operator()(int x, int y);
    int* operator()(double x, double y);

    void erosion(int erosion_size);
    void dilation(int dilation_size);
    void opening(int size);
    void closing(int size);

    void labelConnectDomain();

    void showPixMap();
};


} // namespace scp

#endif // PIX_MAP_HPP