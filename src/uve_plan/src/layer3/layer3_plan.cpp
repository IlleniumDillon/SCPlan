#include "layer3_plan.hpp"

using namespace layer3;
using namespace layer2;
using namespace layer1;

Layer3Plan::Layer3Plan()
{
    /// TODO: cargoCheckPose
    cargoCheckPose.push_back(cv::Point3d(1.2, 0, 0));
    cargoCheckPose.push_back(cv::Point3d(0, 1.2, 0));
    cargoCheckPose.push_back(cv::Point3d(-1.2, 0, 0));
    cargoCheckPose.push_back(cv::Point3d(0, -1.2, 0));
}

void Layer3Plan::setMaxThread(int num)
{
    layer2Plan.setMaxThread(num);
}

void Layer3Plan::setInitGraph(layer1::Layer1GridGraph &freeGraph, layer1::Layer1GridGraph &carryGraph)
{
    this->freeGraph.copyFrom(freeGraph);
    this->carryGraph.copyFrom(carryGraph);
    layer2Plan.setInitGraph(freeGraph, carryGraph);
}

void Layer3Plan::setInitGraph(std::shared_ptr<layer1::Layer1GridGraph> freeGraph, std::shared_ptr<layer1::Layer1GridGraph> carryGraph)
{
    this->freeGraph.copyFrom(freeGraph);
    this->carryGraph.copyFrom(carryGraph);
    layer2Plan.setInitGraph(freeGraph, carryGraph);
}

void Layer3Plan::setWorldDSCP(uvs_message::srv::UvQueryWorld::Response &w)
{
    world = w;
    layer2Plan.setWorldDSCP(w);
}

void Layer3Plan::setFreeExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt)
{
    layer2Plan.setFreeExecuteSpace(max_v, max_w, step_v, step_w, dt);
    layer1Plan.setExecuteSpace(max_v, max_w, step_v, step_w, dt);
}

void Layer3Plan::setCarryExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt)
{
    layer2Plan.setCarryExecuteSpace(max_v, max_w, step_v, step_w, dt);
}

void Layer3Plan::updateGraph(uve_message::msg::UveDynamicStatusList &nstate)
{
    dynamic_state = nstate;
    freeGraph.updateDynamic(nstate);
    carryGraph.updateDynamic(nstate);
    layer2Plan.updateGraph(nstate);
}

Layer3PlanResult Layer3Plan::search(cv::Point3d start, cv::Point3d goal)
{
    Layer3PlanResult result;

    pixMap.fromLayer1Graph(freeGraph);

    if (pixMap(start.x, start.y) == 0 || pixMap(goal.x, goal.y) == 0)
    {
        return Layer3PlanResult();
    }
    if (pixMap(start.x, start.y) == pixMap(goal.x, goal.y))
    {
        layer1Plan.bindGraph(&freeGraph);
        auto ret = layer1Plan.search(start, goal);
        result.success = ret.success;
        result.cost = ret.cost;
        result.path.resize(1);
        result.path[0].path_m = ret.path;
        result.path[0].vw_m = ret.vw;
    }

    connectGraph.edges.clear();
    connectGraph.nodes.clear();

    connectGraph.nodes.resize(pixMap.maxDomainLabel);
    for (int i = 1; i <= pixMap.maxDomainLabel; i++)
    {
        connectGraph.nodes[i-1].domainLabel = i;
    }
    double agent_anchor_center = std::sqrt(std::pow(world.agents[0].anchors[0].x, 2) + std::pow(world.agents[0].anchors[0].y, 2));
    for (int i = 0; i < dynamic_state.list.size(); i++)
    {
        auto& dynamic = dynamic_state.list[i];
        auto it = std::find_if(world.cargos.begin(), world.cargos.end(), [&](const uvs_message::msg::DscpCargo& cargo){
            return cargo.name == dynamic.name;
        });
        if (it == world.cargos.end())
        {
            continue;
        }
        auto& cargo = *it;
        std::vector<double> cargo_anchors_theta;
        for (auto anchor : cargo.anchors)
        {
            cargo_anchors_theta.push_back(std::atan2(anchor.y, anchor.x));
        }
        std::vector<int> connectDomainLabel;
        for (int j = 0; j < cargo.anchors.size(); j++)
        {
            double theta = dynamic.pose.theta + cargo_anchors_theta[j];
            double x = dynamic.pose.x + cargo.anchors[j].x * std::cos(theta) - cargo.anchors[j].y * std::sin(theta)
                        + agent_anchor_center * std::cos(theta);
            double y = dynamic.pose.y + cargo.anchors[j].x * std::sin(theta) + cargo.anchors[j].y * std::cos(theta)
                        + agent_anchor_center * std::sin(theta);
            if (pixMap(x, y) == 0)
            {
                continue;
            }
            connectDomainLabel.push_back(pixMap(x, y));
        }
        if (connectDomainLabel.size() < 2)
        {
            continue;
        }
        connectGraph.edges.push_back(
            Layer3ConnectEdge(
                connectGraph.edges.size(),
                connectGraph.nodes[connectDomainLabel[0]].domainLabel,
                connectGraph.nodes[connectDomainLabel[1]].domainLabel,
                0,
                dynamic.name
            )
        );
        connectGraph.nodes[connectDomainLabel[0]-1].edgeLabels.push_back(connectGraph.edges.size() - 1);
        connectGraph.nodes[connectDomainLabel[1]-1].edgeLabels.push_back(connectGraph.edges.size() - 1);
    }

    std::multimap<double, int> open_set;
    startDomainLabel = pixMap(start.x, start.y);
    goalDomainLabel = pixMap(goal.x, goal.y);
    startAgentPose = start;
    goalAgentPose = goal;

    connectGraph.nodes[startDomainLabel-1].agentPose = start;
    connectGraph.nodes[goalDomainLabel-1].agentPose = goal;
    connectGraph.nodes[startDomainLabel-1].g = 0;
    connectGraph.nodes[startDomainLabel-1].h = 0;
    connectGraph.nodes[startDomainLabel-1].f = 0;
    connectGraph.nodes[startDomainLabel-1].flag = Layer3GraphNodeFlag::IN_OPENSET;
    connectGraph.nodes[startDomainLabel-1].it = open_set.insert(std::make_pair(0, startDomainLabel));
    while (! open_set.empty())
    {
        auto it = open_set.begin();
        int curDomainLabel = it->second;
        open_set.erase(it);
        connectGraph.nodes[curDomainLabel - 1].flag = Layer3GraphNodeFlag::IN_CLOSESET;
        if (curDomainLabel == goalDomainLabel)
        {
            result.success = true;
            result.cost = connectGraph.nodes[curDomainLabel - 1].g;
            while (curDomainLabel != -1)
            {
                result.path.push_back(connectGraph.nodes[curDomainLabel - 1].path);
            }
            std::reverse(result.path.begin(), result.path.end());
            break;
        }

        std::vector<int> neighborDomainLabel;
        std::vector<cv::Point3d> neighborAgentPose;
        std::vector<uve_message::msg::UveDynamicStatusList> neighborDynamic;
        std::vector<Layer2PlanResult> neighborPath;

        getNeighbors(curDomainLabel, neighborDomainLabel, neighborAgentPose, neighborDynamic, neighborPath);

        for (int i = 0; i < neighborDomainLabel.size(); i++)
        {
            int neighborid = neighborDomainLabel[i];
            double g = connectGraph.nodes[curDomainLabel - 1].g + 0;
            double h = 0;
            double f = g + h;
            if (connectGraph.nodes[neighborid - 1].flag != IN_OPENSET)
            {
                connectGraph.nodes[neighborid - 1].g = g;
                connectGraph.nodes[neighborid - 1].h = h;
                connectGraph.nodes[neighborid - 1].f = f;
                connectGraph.nodes[neighborid - 1].parent = curDomainLabel;
                connectGraph.nodes[neighborid - 1].flag = IN_OPENSET;
                connectGraph.nodes[neighborid - 1].agentPose = neighborAgentPose[i];
                connectGraph.nodes[neighborid - 1].dynamic = neighborDynamic[i];
                connectGraph.nodes[neighborid - 1].path = neighborPath[i];
                connectGraph.nodes[neighborid - 1].it = open_set.insert(std::make_pair(f, neighborid));
            }
            else if (g < connectGraph.nodes[neighborid - 1].g)
            {
                open_set.erase(connectGraph.nodes[neighborid - 1].it);
                connectGraph.nodes[neighborid - 1].g = g;
                connectGraph.nodes[neighborid - 1].f = g;
                connectGraph.nodes[neighborid - 1].parent = curDomainLabel;
                connectGraph.nodes[neighborid - 1].agentPose = neighborAgentPose[i];
                connectGraph.nodes[neighborid - 1].dynamic = neighborDynamic[i];
                connectGraph.nodes[neighborid - 1].path = neighborPath[i];
                connectGraph.nodes[neighborid - 1].it = open_set.insert(std::make_pair(connectGraph.nodes[neighborid - 1].f, neighborid));
            }
        }
    }

    return result;
}

void layer3::Layer3Plan::getNeighbors(int curDomainLabel, std::vector<int> &neighbors, std::vector<cv::Point3d> &neighbor_agent_pose, std::vector<uve_message::msg::UveDynamicStatusList> &neighbor_dynamic, std::vector<layer2::Layer2PlanResult> &neighbor_path)
{
    neighbors.clear();
    neighbor_agent_pose.clear();
    neighbor_dynamic.clear();
    neighbor_path.clear();

    for (auto edgeLabel : connectGraph.nodes[curDomainLabel - 1].edgeLabels)
    {
        auto& edge = connectGraph.edges[edgeLabel];
        int label = edge.getOtherNode(curDomainLabel);
        if (connectGraph.nodes[label - 1].flag == IN_CLOSESET)
        {
            continue;
        }
        auto dynamic = connectGraph.nodes[label - 1].dynamic;
        layer2Plan.updateGraph(dynamic);
        auto it = std::find(dynamic.list.begin(), dynamic.list.end(), [&](const uve_message::msg::UveDynamicStatus& d){
                return d.name == edge.name;
            });
        if (it == dynamic.list.end())
        {
            continue;
        }

        cv::Point3d ccur(it->pose.x, it->pose.y, it->pose.theta);

        cv::Point3d start = connectGraph.nodes[curDomainLabel - 1].agentPose;
        std::string cname = edge.name;
        /// TODO: get goal pose
        cv::Point3d cgoal;
        for (auto checkPose : cargoCheckPose)
        {
            auto node = carryGraph(ccur.x + checkPose.x, ccur.y + checkPose.y, ccur.z);
            if (node && !node->collision_static && !node->collision_dynamic)
            {
                cgoal = ccur + checkPose;
                break;
            }
        }
        cv::Point3d agoal;
        if (label == goalDomainLabel)
        {
            agoal = goalAgentPose;
        }
        else
        {
            agoal.x = std::numeric_limits<double>::max();
        }
        auto path = layer2Plan.search(start, cname, cgoal, agoal);
        if (path.success)
        {
            it->pose.x = cgoal.x;
            it->pose.y = cgoal.y;
            it->pose.theta = cgoal.z;
            neighbors.push_back(label);
            neighbor_agent_pose.push_back(path.path_m.back());
            neighbor_dynamic.push_back(dynamic);
            neighbor_path.push_back(path);
        }
    }
}
