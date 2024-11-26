#include "layer3_define.hpp"

using namespace layer3;
using namespace layer2;
using namespace layer1;


void Layer3PixMap::fromLayer1Graph(layer1::Layer1GridGraph &graph)
{
    origin = cv::Point2d(graph.origin.x, graph.origin.y);
    resolution = cv::Point2d(graph.resolution.x, graph.resolution.y);
    size = cv::Point2i(graph.size.x, graph.size.y);
    state_min = cv::Point2d(graph.state_min.x, graph.state_min.y);
    state_max = cv::Point2d(graph.state_max.x, graph.state_max.y);
    data.resize(size.x * size.y);
    int left_pixel = data.size();
    for (int x = 0; x < size.x; x++)
    {
        for (int y = 0; y < size.y; y++)
        {
            data[x + y * size.x] = -1;
            for (int z = 0; z < graph.size.z; z++)
            {
                if (graph(x, y, z)->collision_static || graph(x, y, z)->collision_dynamic)
                {
                    data[x + y * size.x] = 0;
                    left_pixel--;
                    break;
                }
            }
        }
    }

    int domainLabel = 1;

    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    while (left_pixel > 0)
    {
        int x = 0, y = 0;
        for (int i = 0; i < data.size(); i++)
        {
            if (data[i] == -1)
            {
                x = i % size.x;
                y = i / size.x;
                break;
            }
        }
        std::queue<cv::Point2i> q;
        q.push(cv::Point2i(x, y));
        // data[x + y * size.x] = domainLabel;
        // left_pixel--;
        while (!q.empty())
        {
            auto p = q.front();
            q.pop();
            data[p.x + p.y * size.x] = domainLabel;
            left_pixel--;
            for (int i = 0; i < 4; i++)
            {
                int nx = p.x + dx[i];
                int ny = p.y + dy[i];
                if (nx >= 0 && nx < size.x && ny >= 0 && ny < size.y && data[nx + ny * size.x] == -1)
                {
                    // data[nx + ny * size.x] = domainLabel;
                    // left_pixel--;
                    q.push(cv::Point2i(nx, ny));
                }
            }
        }
        domainLabel++;
    }
    maxDomainLabel = domainLabel - 1;
}

int Layer3PixMap::operator()(int x, int y)
{
    if (x < 0 || x >= size.x || y < 0 || y >= size.y)
    {
        return -1;
    }
    return data[x + y * size.x];
}

int Layer3PixMap::operator()(double x, double y)
{
    int ix = (x - state_min.x) / resolution.x;
    int iy = (y - state_min.y) / resolution.y;
    return (*this)(ix, iy);
}

void Layer3PixMap::copyFrom(Layer3PixMap &other)
{
    origin = other.origin;
    resolution = other.resolution;
    size = other.size;
    state_min = other.state_min;
    state_max = other.state_max;
    data = other.data;
    maxDomainLabel = other.maxDomainLabel;
}
