#include "layer2_plan.hpp"

using namespace layer2;

void Layer2Plan::setMaxThread(int num)
{
    this->max_thread = num;
    plans = std::vector<layer1::Layer1Plan>(num);
    freeGraphs = std::vector<layer1::Layer1GridGraph>(num);
    carryGraphs = std::vector<layer1::Layer1GridGraph>(num);
}

void layer2::Layer2Plan::setInitGraph(layer1::Layer1GridGraph &freeGraph, layer1::Layer1GridGraph &carryGraph)
{
    for (int i = 0; i < max_thread; i++)
    {
        freeGraphs[i].copyFrom(freeGraph);
        carryGraphs[i].copyFrom(carryGraph);
    }
    std::cout << "freeGraph size: " << freeGraph.size.x << " " << freeGraph.size.y << " " << freeGraph.size.z << std::endl;
    std::cout << "carryGraph size: " << carryGraph.size.x << " " << carryGraph.size.y << " " << carryGraph.size.z << std::endl;
}

void layer2::Layer2Plan::setWorldDSCP(uvs_message::srv::UvQueryWorld::Response &w)
{
    world = w;
}

void Layer2Plan::setFreeExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt)
{
    freeConfig.clear();
    freeConfig = {max_v, max_w, (double)step_v, (double)step_w, dt};
}

void Layer2Plan::setCarryExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt)
{
    carryConfig.clear();
    carryConfig = {max_v, max_w, (double)step_v, (double)step_w, dt};
}

void layer2::Layer2Plan::updateGraph(uve_message::msg::UveDynamicStatusList &nstate)
{
    for (int i = 0; i < max_thread; i++)
    {
        freeGraphs[i].updateDynamic(nstate);
        carryGraphs[i].updateDynamic(nstate);
    }
}

Layer2PlanResult Layer2Plan::search(cv::Point3d Astate, std::string Cname, cv::Point3d Cgoal, cv::Point3d Agoal)
{
    auto& freegraph0 = freeGraphs[0];
    cur_Cname = Cname;
    cur_Cgoal = Cgoal;
    std::cout << "searching " << Cname << std::endl;
    // 退化情况，不携带物体
    if (Cname == "")
    {
        layer1::Layer1Plan plan;
        plan.setExecuteSpace(freeConfig[0], freeConfig[1], freeConfig[2], freeConfig[3], freeConfig[4]);
        plan.bindGraph(&freegraph0);
        auto ret = plan.search(Astate, Agoal);
        Layer2PlanResult result;
        if (ret.success)
        {
            result.success = true;
            result.cost = ret.cost;
            result.path_m = ret.path;
            result.vw_m = ret.vw;
        }
        return result;
    }
    std::cout << "searched " << Cname << std::endl;
    // 正常情况，携带物体
    // 找不到目标物体
    auto targetCargoIt = freegraph0.dynamic_map.find(Cname);
    if (targetCargoIt == freegraph0.dynamic_map.end())
    {
        return Layer2PlanResult();
    }

    // 找到目标物体
    auto& targetCargo = world.cargos[targetCargoIt->second];
    int plan_num = targetCargo.anchors.size();
    cv::Point3d Cstart = freegraph0.dynamic_pose[targetCargoIt->second];
    std::cout << "Cstart: " << Cstart.x << " " << Cstart.y << " " << Cstart.z << std::endl;
    std::vector<cv::Point3d> CAstart_list(plan_num);
    std::vector<cv::Point3d> CAgoal_list(plan_num);
    std::vector<double> cargo_anchors_theta;
    for (auto& p : targetCargo.anchors)
    {
        cargo_anchors_theta.push_back(
            std::atan2(p.y, p.x)
        );
    }
    double agent_anchor_center = std::sqrt(std::pow(world.agents[0].anchors[0].x, 2) + std::pow(world.agents[0].anchors[0].y, 2));
    // 计算目标初始状态下时对应的智能体位姿
    for (int i = 0; i < plan_num; i++)
    {
        CAstart_list[i].z = Cstart.z + cargo_anchors_theta[i];
        CAstart_list[i].x = Cstart.x + targetCargo.anchors[i].x * std::cos(Cstart.z) - targetCargo.anchors[i].y * std::sin(Cstart.z)
                            + agent_anchor_center * std::cos(CAstart_list[i].z);
        CAstart_list[i].y = Cstart.y + targetCargo.anchors[i].x * std::sin(Cstart.z) + targetCargo.anchors[i].y * std::cos(Cstart.z)
                            + agent_anchor_center * std::sin(CAstart_list[i].z);
        CAstart_list[i].z = M_PI - CAstart_list[i].z;
        if (CAstart_list[i].z > M_PI)
        {
            CAstart_list[i].z -= 2 * M_PI;
        }
        if (CAstart_list[i].z <= -M_PI)
        {
            CAstart_list[i].z += 2 * M_PI;
        }
    }
    // 计算目标终末状态下时对应的智能体位姿
    for (int i = 0; i < plan_num; i++)
    {
        CAgoal_list[i].z = Cgoal.z + cargo_anchors_theta[i];
        CAgoal_list[i].x = Cgoal.x + targetCargo.anchors[i].x * std::cos(Cgoal.z) - targetCargo.anchors[i].y * std::sin(Cgoal.z)
                            + agent_anchor_center * std::cos(CAgoal_list[i].z);
        CAgoal_list[i].y = Cgoal.y + targetCargo.anchors[i].x * std::sin(Cgoal.z) + targetCargo.anchors[i].y * std::cos(Cgoal.z)
                            + agent_anchor_center * std::sin(CAgoal_list[i].z);
        CAgoal_list[i].z = M_PI - CAgoal_list[i].z;
        if (CAgoal_list[i].z > M_PI)
        {
            CAgoal_list[i].z -= 2 * M_PI;
        }
        if (CAgoal_list[i].z <= -M_PI)
        {
            CAgoal_list[i].z += 2 * M_PI;
        }
    }
    // 开辟线程进行规划
    // 等待线程结束
    // 收集结果，选择最优解
    int cur_process = 0;
    while (cur_process < plan_num)
    {
        // for (int i = 0; i < max_thread; i++)
        // {
        //     futures.push_back(std::async(std::launch::async, &Layer2Plan::searchThread, this, i, Astate, CAstart_list[cur_process], CAgoal_list[cur_process], Agoal));
        //     cur_process++;
        //     if (cur_process >= plan_num)
        //     {
        //         break;
        //     }
        // }
        // for (int i = 0; i < max_thread; i++)
        // {
        //     if (futures[i].valid())
        //     {
        //         auto ret = futures[i].get();
        //         if (ret.success)
        //         {
        //             results.insert(std::make_pair(ret.cost, ret));
        //         }
        //     }
        // }
        auto ret = searchThread(0, Astate, CAstart_list[cur_process], CAgoal_list[cur_process], Agoal);
        if (ret.success)
        {
            results.insert(std::make_pair(ret.cost, ret));
        }
        cur_process++;
    }

    if (results.empty())
    {
        return Layer2PlanResult();
    }
    auto result = results.begin()->second;
    results.clear();
    return result;
}

Layer2PlanResult Layer2Plan::searchThread(int id, cv::Point3d Astart, cv::Point3d CAstart, cv::Point3d CAgoal, cv::Point3d Agoal)
{
    auto time_start = std::chrono::high_resolution_clock::now();
    Layer2PlanResult result;
    plans[id].setExecuteSpace(freeConfig[0], freeConfig[1], freeConfig[2], freeConfig[3], freeConfig[4]);
    std::cout << "[" << id << "]" << "freeConfig: " << freeConfig[0] << " " << freeConfig[1] << " " << freeConfig[2] << " " << freeConfig[3] << " " << freeConfig[4] << std::endl;
    plans[id].bindGraph(&freeGraphs[id]);
    std::cout << "[" << id << "]" << "Astart: " << Astart.x << " " << Astart.y << " " << Astart.z << std::endl;
    std::cout << "[" << id << "]" << "CAstart: " << CAstart.x << " " << CAstart.y << " " << CAstart.z << std::endl;
    auto ret = plans[id].search(Astart, CAstart);
    std::cout << "[" << id << "]" << "ret: " << ret.success << " " << ret.iterations << std::endl;
    if (!ret.success)
    {
        return Layer2PlanResult();
    }
    std::cout << "[" << id << "]" << 159 << std::endl;
    result.cost = ret.cost;
    result.path_m = ret.path;
    result.vw_m = ret.vw;

    carryGraphs[id].ignoreDynamicCollision(cur_Cname);
    plans[id].setExecuteSpace(carryConfig[0], carryConfig[1], carryConfig[2], carryConfig[3], carryConfig[4]);
    plans[id].bindGraph(&carryGraphs[id]);
    std::cout << "[" << id << "]" << "CAgoal: " << CAgoal.x << " " << CAgoal.y << " " << CAgoal.z << std::endl;
    ret = plans[id].search(CAstart, CAgoal);
    std::cout << "[" << id << "]" << "ret: " << ret.success << " " << ret.iterations << std::endl;
    if (!ret.success)
    {
        return Layer2PlanResult();
    }
    result.cost += ret.cost;
    result.path_c = ret.path;
    result.vw_c = ret.vw;

    /// 更新地图
    uve_message::msg::UveDynamicStatusList updates;
    uve_message::msg::UveDynamicStatus update;
    update.name = cur_Cname;
    update.pose.x = cur_Cgoal.x;
    update.pose.y = cur_Cgoal.y;
    update.pose.theta = cur_Cgoal.z;
    updates.list.push_back(update);
    freeGraphs[id].updateDynamic(updates);

    plans[id].setExecuteSpace(freeConfig[0], freeConfig[1], freeConfig[2], freeConfig[3], freeConfig[4]);
    plans[id].bindGraph(&freeGraphs[id]);
    ret = plans[id].search(CAgoal, Agoal);
    if (!ret.success)
    {
        return Layer2PlanResult();
    }
    result.cost += ret.cost;
    result.path_a = ret.path;
    result.vw_a = ret.vw;
    result.success = true;
    auto time_end = std::chrono::high_resolution_clock::now();
    result.planTime = std::chrono::duration<double, std::nano>(time_end - time_start).count();
    return result;
}
