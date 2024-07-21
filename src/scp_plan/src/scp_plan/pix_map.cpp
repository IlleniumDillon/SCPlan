#include "pix_map.hpp"

using namespace scp;

PixMap::PixMap()
{
    width = 0;
    height = 0;
    domain_num = 0;
    resolution = 0;
    pix_map = nullptr;
}

PixMap::PixMap(GridMap &grid_map)
{
    this->width = grid_map.width;
    this->height = grid_map.height;
    this->domain_num = 0;
    this->resolution = grid_map.positionResolution;
    this->maxX = grid_map.maxX;
    this->maxY = grid_map.maxY;
    this->minX = grid_map.minX;
    this->minY = grid_map.minY;
    pix_map = new int *[height];
    for (int i = 0; i < height; i++)
    {
        pix_map[i] = new int[width];
        for (int j = 0; j < width; j++)
        {
            if (grid_map(i,j).occupied)
            {
                pix_map[i][j] = PIX_OCCUPIED;
            }
            else
            {
                pix_map[i][j] = PIX_FREE;
            }
        }
    }
}

PixMap::PixMap(const PixMap &pix_map)
{
    if (this->pix_map != nullptr)
    {
        for (int i = 0; i < height; i++)
        {
            delete[] this->pix_map[i];
        }
        delete[] this->pix_map;
    }
    this->width = pix_map.width;
    this->height = pix_map.height;
    this->domain_num = pix_map.domain_num;
    this->resolution = pix_map.resolution;
    this->maxX = pix_map.maxX;
    this->maxY = pix_map.maxY;
    this->minX = pix_map.minX;
    this->minY = pix_map.minY;

    this->pix_map = new int *[height];
    for (int i = 0; i < height; i++)
    {
        this->pix_map[i] = new int[width];
        for (int j = 0; j < width; j++)
        {
            this->pix_map[i][j] = pix_map.pix_map[i][j];
        }
    }
}

PixMap &PixMap::operator=(const PixMap &pix_map)
{
    if (this == &pix_map)
    {
        return *this;
    }
    if (this->pix_map != nullptr)
    {
        for (int i = 0; i < height; i++)
        {
            delete[] this->pix_map[i];
        }
        delete[] this->pix_map;
    }
    this->width = pix_map.width;
    this->height = pix_map.height;
    this->domain_num = pix_map.domain_num;
    this->resolution = pix_map.resolution;
    this->maxX = pix_map.maxX;
    this->maxY = pix_map.maxY;
    this->minX = pix_map.minX;
    this->minY = pix_map.minY;

    this->pix_map = new int *[height];
    for (int i = 0; i < height; i++)
    {
        this->pix_map[i] = new int[width];
        for (int j = 0; j < width; j++)
        {
            this->pix_map[i][j] = pix_map.pix_map[i][j];
        }
    }
}

PixMap::~PixMap()
{
    if (this->pix_map != nullptr)
    {
        for (int i = 0; i < height; i++)
        {
            delete[] this->pix_map[i];
        }
        delete[] this->pix_map;
    }
}

void PixMap::convertGridMap(GridMap &grid_map)
{
    if (this->pix_map != nullptr)
    {
        for (int i = 0; i < height; i++)
        {
            delete[] this->pix_map[i];
        }
        delete[] this->pix_map;
    }
    this->width = grid_map.width;
    this->height = grid_map.height;
    this->domain_num = 0;
    this->resolution = grid_map.positionResolution;
    this->maxX = grid_map.maxX;
    this->maxY = grid_map.maxY;
    this->minX = grid_map.minX;
    this->minY = grid_map.minY;

    pix_map = new int *[height];
    for (int i = 0; i < height; i++)
    {
        pix_map[i] = new int[width];
        for (int j = 0; j < width; j++)
        {
            if (grid_map(i,j).occupied)
            {
                pix_map[i][j] = PIX_OCCUPIED;
            }
            else
            {
                pix_map[i][j] = PIX_FREE;
            }
        }
    }
}

int *PixMap::operator()(int x, int y)
{
    if (x < 0 || x >= width || y < 0 || y >= height)
    {
        return nullptr;
    }
    return &pix_map[y][x];
}

int *PixMap::operator()(double x, double y)
{
    return (*this)((int)((x - minX) / resolution), (int)((y - minY) / resolution));
}

void PixMap::erosion(int erosion_size)
{
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (pix_map[i][j] != PIX_OCCUPIED)
            {
                continue;
            }
            for (int k = -erosion_size; k <= erosion_size; k++)
            {
                for (int l = -erosion_size; l <= erosion_size; l++)
                {
                    if (i + k < 0 || i + k >= height || j + l < 0 || j + l >= width)
                    {
                        continue;
                    }
                    if (pix_map[i + k][j + l] == PIX_FREE)
                    {
                        pix_map[i + k][j + l] = PIX_TEMP;
                    }
                }
            }
        }
    }
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (pix_map[i][j] == PIX_TEMP)
            {
                pix_map[i][j] = PIX_OCCUPIED;
            }
        }
    }
}

void PixMap::dilation(int dilation_size)
{
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (pix_map[i][j] != PIX_FREE)
            {
                continue;
            }
            for (int k = -dilation_size; k <= dilation_size; k++)
            {
                for (int l = -dilation_size; l <= dilation_size; l++)
                {
                    if (i + k < 0 || i + k >= height || j + l < 0 || j + l >= width)
                    {
                        continue;
                    }
                    if (pix_map[i + k][j + l] == PIX_OCCUPIED)
                    {
                        pix_map[i + k][j + l] = PIX_TEMP;
                    }
                }
            }
        }
    }
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (pix_map[i][j] == PIX_TEMP)
            {
                pix_map[i][j] = PIX_FREE;
            }
        }
    }
}

void PixMap::opening(int size)
{
    dilation(size);
    erosion(size);
}

void PixMap::closing(int size)
{
    erosion(size);
    dilation(size);
}

void PixMap::labelConnectDomain()
{
    domain_num = 0;
    // int close_cont = 0;

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (pix_map[i][j] != PIX_FREE)
            {
                continue;
            }
            domain_num++;
            // RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Label connect domain: %d.", domain_num);
            std::queue<std::pair<int, int>> q;
            q.push(std::make_pair(i, j));
            while (!q.empty())
            {
                std::pair<int, int> p = q.front();
                q.pop();
                int x = p.second;
                int y = p.first;
                pix_map[y][x] = domain_num - 1;
                // close_cont++;
                // RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "queue size: %d, %d.", q.size(), close_cont);
                // showPixMap();

                for (int k = -1; k <= 1; k++)
                {
                    for (int l = -1; l <= 1; l++)
                    {
                        if (
                            (k == 0 && l == 0) //||
                            // (k != 0 && l != 0)
                        )
                        {
                            continue;
                        }
                        if (x + l < 0 || x + l >= width || y + k < 0 || y + k >= height)
                        {
                            continue;
                        }
                        if (pix_map[y + k][x + l] == PIX_FREE)
                        {
                            pix_map[y + k][x + l] = PIX_OPENING;
                            q.push(std::make_pair(y + k, x + l));
                        }
                    }
                }
            }
        }
    }
}

void PixMap::showPixMap()
{
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    // color list
    std::vector<cv::Vec3b> color_list;
    for (int i = 0; i < domain_num; i++)
    {
        cv::Vec3b color;
        color[0] = rand() % 256;
        color[1] = rand() % 256;
        color[2] = rand() % 256;
        color_list.push_back(color);
    }

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (pix_map[i][j] == PIX_OCCUPIED)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
            else if (pix_map[i][j] == PIX_FREE)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
            else
            {
                img.at<cv::Vec3b>(i, j) = color_list[pix_map[i][j]];
            }
        }
    }
    cv::resize(img, img, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);
    cv::imshow("PixMap", img);
    cv::waitKey(0);
}
