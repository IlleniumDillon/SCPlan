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
    // pix_map = new int *[height];
    // for (int i = 0; i < height; i++)
    // {
    //     pix_map[i] = new int[width];
    //     for (int j = 0; j < width; j++)
    //     {
    //         if (grid_map(i,j).occupied)
    //         {
    //             pix_map[i][j] = PIX_OCCUPIED;
    //         }
    //         else
    //         {
    //             pix_map[i][j] = PIX_FREE;
    //         }
    //     }
    // }
    pix_map = new int *[width];
    for (int i = 0; i < width; i++)
    {
        pix_map[i] = new int[height];
        for (int j = 0; j < height; j++)
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

    // this->pix_map = new int *[height];
    // for (int i = 0; i < height; i++)
    // {
    //     this->pix_map[i] = new int[width];
    //     for (int j = 0; j < width; j++)
    //     {
    //         this->pix_map[i][j] = pix_map.pix_map[i][j];
    //     }
    // }
    this->pix_map = new int *[width];
    for (int i = 0; i < width; i++)
    {
        this->pix_map[i] = new int[height];
        for (int j = 0; j < height; j++)
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

    // this->pix_map = new int *[height];
    // for (int i = 0; i < height; i++)
    // {
    //     this->pix_map[i] = new int[width];
    //     for (int j = 0; j < width; j++)
    //     {
    //         this->pix_map[i][j] = pix_map.pix_map[i][j];
    //     }
    // }
    this->pix_map = new int *[width];
    for (int i = 0; i < width; i++)
    {
        this->pix_map[i] = new int[height];
        for (int j = 0; j < height; j++)
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

    // pix_map = new int *[height];
    // for (int i = 0; i < height; i++)
    // {
    //     pix_map[i] = new int[width];
    //     for (int j = 0; j < width; j++)
    //     {
    //         if (grid_map(i,j).occupied)
    //         {
    //             pix_map[i][j] = PIX_OCCUPIED;
    //         }
    //         else
    //         {
    //             pix_map[i][j] = PIX_FREE;
    //         }
    //     }
    // }
    pix_map = new int *[width];
    for (int i = 0; i < width; i++)
    {
        pix_map[i] = new int[height];
        for (int j = 0; j < height; j++)
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
    return &pix_map[x][y];
}

int *PixMap::operator()(double x, double y)
{
    return (*this)((int)((x - minX) / resolution), (int)((y - minY) / resolution));
}

void PixMap::erosion(int erosion_size)
{
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            if (pix_map[i][j] != PIX_OCCUPIED)
            {
                continue;
            }
            for (int k = -erosion_size; k <= erosion_size; k++)
            {
                for (int l = -erosion_size; l <= erosion_size; l++)
                {
                    if (i + k < 0 || i + k >= width || j + l < 0 || j + l >= height)
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
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
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
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            if (pix_map[i][j] != PIX_FREE)
            {
                continue;
            }
            for (int k = -dilation_size; k <= dilation_size; k++)
            {
                for (int l = -dilation_size; l <= dilation_size; l++)
                {
                    if (i + k < 0 || i + k >= width || j + l < 0 || j + l >= height)
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
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
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

    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            if (pix_map[i][j] != PIX_FREE)
            {
                continue;
            }
            domain_num++;
            int x_sum = 0;
            int y_sum = 0;
            int count = 0;
            std::queue<std::pair<int, int>> q;
            q.push(std::make_pair(i, j));
            while (!q.empty())
            {
                std::pair<int, int> p = q.front();
                q.pop();
                int x = p.first;
                int y = p.second;
                x_sum += x;
                y_sum += y;
                count++;
                pix_map[x][y] = domain_num - 1;

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
                        if (pix_map[x + l][y + k] == PIX_FREE)
                        {
                            pix_map[x + l][y + k] = PIX_OPENING;
                            q.push(std::make_pair(x + l, y + k));
                        }
                    }
                }
            }
            // get domain center
            Point center;
            center.x = (double)x_sum / count * resolution + minX;
            center.y = (double)y_sum / count * resolution + minY;
            domain_center.push_back(center);
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

    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
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

    for (int i = 0; i < domain_center.size(); i++)
    {
        cv::circle(img, cv::Point((domain_center[i].x - minX) / resolution, (domain_center[i].y - minY) / resolution), 3, cv::Scalar(0, 0, 255), -1);
    }

    cv::resize(img, img, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);
    cv::imshow("PixMap", img);
    cv::waitKey(0);
}
