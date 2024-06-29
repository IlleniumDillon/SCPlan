#include "test.hpp"
#include "json/json.h"

ResolutionConfig loadConfig(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open config file." << std::endl;
        return ResolutionConfig();
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(file, root))
    {
        std::cerr << "Error: Cannot parse config file." << std::endl;
        return ResolutionConfig();
    }

    file.close();

    ResolutionConfig config;
    if (!root["solveResolution"].isNull())
        config.solveResolution = root["solveResolution"].asDouble();
    if (!root["distanceMapResolution"].isNull())
        config.distanceMapResolution = root["distanceMapResolution"].asDouble();

    return config;
}

TaskLowLevel loadTask(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open task file." << std::endl;
        return TaskLowLevel();
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(file, root))
    {
        std::cerr << "Error: Cannot parse task file." << std::endl;
        return TaskLowLevel();
    }

    file.close();

    if(root["start"].isNull() || 
        root["theta0"].isNull() ||
        root["goal"].isNull() ||
        root["theta1"].isNull() ||
        root["V"].isNull() ||
        root["W"].isNull() ||
        root["dt"].isNull())
    {
        std::cerr << "Error: Task file is not complete." << std::endl;
        return TaskLowLevel();
    }

    TaskLowLevel task;
    task.start.x = root["start"][0].asDouble();
    task.start.y = root["start"][1].asDouble();
    task.theta0 = root["theta0"].asDouble();
    task.goal.x = root["goal"][0].asDouble();
    task.goal.y = root["goal"][1].asDouble();
    task.theta1 = root["theta1"].asDouble();
    task.V = root["V"].asDouble();
    task.W = root["W"].asDouble();
    task.dt = root["dt"].asDouble();

    return task;
}

TaskCarry loadTaskCarry(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open task file." << std::endl;
        return TaskCarry();
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(file, root))
    {
        std::cerr << "Error: Cannot parse task file." << std::endl;
        return TaskCarry();
    }

    file.close();

    if (
        root["agentStart"].isNull() ||
        root["agentTheta0"].isNull() ||
        root["elementId"].isNull() ||
        root["elementTargetPosition"].isNull() ||
        root["elementTargetTheta"].isNull() ||
        root["agentGoal"].isNull() ||
        root["agentTheta1"].isNull() ||
        root["V"].isNull() ||
        root["W"].isNull() ||
        root["dt"].isNull()
    )
    {
        std::cerr << "Error: Task file is not complete." << std::endl;
        return TaskCarry();
    }

    TaskCarry task;

    task.agentStart.x=root["agentStart"][0].asDouble();
    task.agentStart.y=root["agentStart"][1].asDouble();
    task.agentTheta0=root["agentTheta0"].asDouble();
    task.elementId=root["elementId"].asInt();
    task.elementTargetPosition.x=root["elementTargetPosition"][0].asDouble();
    task.elementTargetPosition.y=root["elementTargetPosition"][1].asDouble();
    task.elementTargetTheta=root["elementTargetTheta"].asDouble();
    task.agentGoal.x=root["agentGoal"][0].asDouble();
    task.agentGoal.y=root["agentGoal"][1].asDouble();
    task.agentTheta1=root["agentTheta1"].asDouble();
    task.V=root["V"].asDouble();
    task.W=root["W"].asDouble();
    task.dt=root["dt"].asDouble();

    return task;
}

void MannualTest()
{
    ResolutionConfig config = loadConfig(CONFIG_PATH);
    ElementMap map(MAPFILE_PATH);
    TaskLowLevel task = loadTask(TASK_PATH);

    DistanceElementMap distanceMap(map, config.distanceMapResolution);

    Kinematics kinematicsq(task.V, task.W, task.dt);
    Kinematics kinematicsw(task.V, 0, task.dt);
    Kinematics kinematicse(task.V, -task.W, task.dt);

    CoordD position0 = task.start;
    double theta0 = task.theta0;
    CoordD position1;
    double theta1;
    map.agent.setGeometry(position0, theta0);

    cv::namedWindow("Map", cv::WINDOW_NORMAL);

    int targetWidth = 900;
    int targetHeight = 900;

    double scaleX = targetWidth / (map.maxX - map.minX);
    double scaleY = targetHeight / (map.maxY - map.minY);
    double scale = std::min(scaleX, scaleY);

    targetHeight = (map.maxY - map.minY) * scale;
    targetWidth = (map.maxX - map.minX) * scale;

    bool running = true;
    while(running)
    {
        auto checkList = distanceMap[position0]->elementIds;
        bool collision = false;
        for (auto &e : map.elements)
        {
            if (checkList.count(e.id) > 0)
            {
                if (map.agent.shape.isIntersect(e.shape))
                {
                    collision = true;
                    break;
                }
            }
        }

        cv::Mat img(targetHeight, targetWidth, CV_8UC3, cv::Scalar(255, 255, 255));

        // std::vector<std::vector<cv::Point>> contours;
        for (Element &e : map.elements)
        {
            std::vector<cv::Point> points;
            for (CoordD &c : e.shape.vertices)
            {
                CoordD p = c.transform(&map.visualizeCS) * scale;
                points.push_back(cv::Point(p.x, p.y));
            }
            // contours.push_back(points);
            if (checkList.count(e.id) > 0)
                cv::fillConvexPoly(img, points, cv::Scalar(128,128,128));
            else
                cv::fillConvexPoly(img, points, cv::Scalar(0, 0, 0));
        }
        // cv::fillPoly(img, contours, cv::Scalar(0, 0, 0));

        std::vector<cv::Point> points;
        for (CoordD &c : map.agent.shape.vertices)
        {
            CoordD p = c.transform(&map.visualizeCS) * scale;
            points.push_back(cv::Point(p.x, p.y));
        }
        if (collision)
            cv::fillConvexPoly(img, points, cv::Scalar(0, 0, 255));
        else
            cv::fillConvexPoly(img, points, cv::Scalar(0, 255, 0));

        cv::imshow("Map", img);

        int key = cv::waitKey(0);
        switch (key)
        {
        case 'q':
        {
            kinematicsq.update(position0, theta0, position1, theta1);
            map.agent.setGeometry(position1, theta1);
            position0 = position1;
            theta0 = theta1;
            break;
        }
        case 'w':
        {
            kinematicsw.update(position0, theta0, position1, theta1);
            map.agent.setGeometry(position1, theta1);
            position0 = position1;
            theta0 = theta1;
            break;
        }
        case 'e':
        {
            kinematicse.update(position0, theta0, position1, theta1);
            map.agent.setGeometry(position1, theta1);
            position0 = position1;
            theta0 = theta1;
            break;
        }
        case 'x':
        {
            running = false;
            break;
        }
        default:
            break;
        }
    }
}

void HybridAStarPlan()
{
    ResolutionConfig config = loadConfig(CONFIG_PATH);
    TaskLowLevel task = loadTask(TASK_PATH);
    ElementMap map(MAPFILE_PATH);
    HybridAStar planner(map,config.solveResolution,config.distanceMapResolution);

    PlanResult result = planner.plan(task);
    std::cout << "Result: " << result.success << "\n";
    std::cout << "Iterations: " << result.iterations << "\n";
    std::cout << "Time Cost: " << result.timeCost << "\n";
    drawDynamic(map, result);

    task.start = result.actions.back().to;
    task.theta0 = result.actions.back().theta1;
    task.goal = CoordD(-4,3);
    task.theta1 = 0;

    result = planner.plan(task);
    std::cout << "Result: " << result.success << "\n";
    std::cout << "Iterations: " << result.iterations << "\n";
    std::cout << "Time Cost: " << result.timeCost << "\n";
    drawDynamic(map, result);

    // cv::waitKey(0);
}

void CarryPlan()
{
    ResolutionConfig config = loadConfig(CONFIG_PATH);
    TaskCarry task = loadTaskCarry(TASK_PATH);
    ElementMap map(MAPFILE_PATH);
    CarryPlanner planner(map,config.solveResolution,config.distanceMapResolution);
    PlanResult result = planner.plan(task);

    std::cout << "Result: " << result.success << "\n";
    std::cout << "Iterations: " << result.iterations << "\n";
    std::cout << "Time Cost: " << result.timeCost << "\n";
    drawDynamic(map, result);
}
