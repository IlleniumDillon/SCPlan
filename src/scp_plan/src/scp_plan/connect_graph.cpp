#include "connect_graph.hpp"

using namespace scp;

ConnectGraph::ConnectGraph()
{
}

void ConnectGraph::config(double radius)
{
    this->radius = radius;
}

void ConnectGraph::convertGridMap(GridMap &grid_map)
{
    //RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Convert grid map to pix map.");
    pix_map = PixMap(grid_map);
    //pix_map.showPixMap();
    //RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Closing pix map: %d.", (int)(radius / pix_map.resolution));
    pix_map.closing((int)(radius / pix_map.resolution));
    //pix_map.showPixMap();
    //RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Label connect domain.");
    pix_map.labelConnectDomain();
    // pix_map.showPixMap();
    //RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Convert grid map to pix map done.");
}

void ConnectGraph::buildGraph(std::vector<Element> &dynamic_elements)
{
    // generate node according to pix_map
    for (int i = 0; i < pix_map.domain_num; i++)
    {
        ConnectNode node(i, pix_map.domain_center[i]);
        // RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Node %d: %f, %f", i, pix_map.domain_center[i].x, pix_map.domain_center[i].y);
        nodes.push_back(node);
    }
    // RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Node size: %d", nodes.size());

    // generate edge according to dynamic_elements
    for (auto &e : dynamic_elements)
    {
        std::set<int> connect_domain;
        for (int i = 0; i < e.currentAnchors.size(); i++)
        {
            // RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Element %d: %f, %f", i, e.currentAnchors[i].x, e.currentAnchors[i].y);
            int x = (e.currentAnchors[i].x - pix_map.minX) / pix_map.resolution;
            int y = (e.currentAnchors[i].y - pix_map.minY) / pix_map.resolution;
            
            for (int k = -1; k <= 1; k++)
            {
                for (int l = -1; l <= 1; l++)
                {
                    if (x + l < 0 || x + l >= pix_map.width || y + k < 0 || y + k >= pix_map.height)
                    {
                        continue;
                    }
                    int domain = pix_map.pix_map[x + l][y + k];
                    if (domain >= 0)
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "%d,%d,%d.", x + l, y + k, domain);
                        connect_domain.insert(domain);
                    }
                }
            }
        }
        // RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "connect_domain:%d", connect_domain.size());

        // generate edge according to connect_domain
        for (int i = 0; i < connect_domain.size(); i++)
        {
            for (int j = i + 1; j < connect_domain.size(); j++)
            {
                int domain1 = *(std::next(connect_domain.begin(), i));
                int domain2 = *(std::next(connect_domain.begin(), j));

                Point p1 = pix_map.domain_center[domain1];
                Point p2 = pix_map.domain_center[domain2];
                Point p3;
                p3.x = e.pose.x;
                p3.y = e.pose.y;
                double d1 = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2));
                double d2 = sqrt(pow(p2.x - p3.x, 2) + pow(p2.y - p3.y, 2));
                double d = d1 + d2;
                ConnectEdge edge(e.id, domain1, domain2, d);
                edges.push_back(edge);

                nodes[domain1].edges.push_back(edges.size() - 1);
                nodes[domain2].edges.push_back(edges.size() - 1);
            }
        }
    }
}

void ConnectGraph::dijkstra(Point start, Point goal, ConnectRoutes& path)
{
    path.clear();

    int* pstart_domain = pix_map(start.x, start.y);
    int* pgoal_domain = pix_map(goal.x, goal.y);

    if (pstart_domain == nullptr || pgoal_domain == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("scp_plan"), "Start or goal domain is nullptr.");
        return;
    }

    int start_domain = *pstart_domain;
    int goal_domain = *pgoal_domain;

    std::multimap<double, int> open;

    nodes[start_domain].position = start;
    nodes[start_domain].g = 0;
    nodes[start_domain].state = IN_OPENSET;
    nodes[start_domain].iterator = open.insert(std::make_pair(nodes[start_domain].g, start_domain));

    nodes[goal_domain].position = goal;

    int current;

    while (!open.empty())
    {
        current = open.begin()->second;
        if (current == goal_domain)
        {
            while (current != start_domain)
            {
                int parent = nodes[current].parent;
                int edge = nodes[current].edge;
                path.push_back(ConnectRoute(nodes[parent].position, nodes[current].position, edges[edge].id));
                current = parent;
            }
            std::reverse(path.begin(), path.end());
            return;
        }
        open.erase(open.begin());
        nodes[current].state = IN_CLOSESET;

        for (auto &e : nodes[current].edges)
        {
            int next = edges[e].getOtherIndex(current);
            if (nodes[next].state == IN_CLOSESET)
            {
                continue;
            }
            double new_g = nodes[current].g + edges[e].weight;
            if (nodes[next].state == NOT_VISITED)
            {
                nodes[next].g = new_g;
                nodes[next].parent = current;
                nodes[next].edge = e;
                nodes[next].state = IN_OPENSET;
                nodes[next].iterator = open.insert(std::make_pair(nodes[next].g, next));
            }
            else if (new_g < nodes[next].g)
            {
                open.erase(nodes[next].iterator);
                nodes[next].g = new_g;
                nodes[next].parent = current;
                nodes[next].edge = e;
                nodes[next].iterator = open.insert(std::make_pair(nodes[next].g, next));
            }
        }
    }
}

ConnectNode::ConnectNode()
{
}

ConnectNode::ConnectNode(int index, Point position)
{
    this->index = index;
    this->position = position;
}

ConnectEdge::ConnectEdge()
{
    this->index1 = -1;
    this->index2 = -1;
    this->weight = 0;
}

ConnectEdge::ConnectEdge(int id, int index1, int index2, double weight)
{
    this->id = id;
    this->index1 = index1;
    this->index2 = index2;
    this->weight = weight;
}

int ConnectEdge::getOtherIndex(int index)
{
    return index == index1 ? index2 : index1;
}
