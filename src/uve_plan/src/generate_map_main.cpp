#include <rclcpp/rclcpp.hpp>

#include "layer1_node.hpp"

using namespace std::chrono_literals;


class GeneratorNode : public rclcpp::Node
{
public:
    GeneratorNode() : Node("generator")
    {
        declare_parameter("save_path");
        declare_parameter("free_name");
        declare_parameter("carry_name");
        declare_parameter("resolution");
        savePath = get_parameter("save_path").as_string();
        freeMapName = get_parameter("free_name").as_string();
        carryMapName = get_parameter("carry_name").as_string();
        std::vector<double> res_ = get_parameter("resolution").as_double_array();
        res = cv::Point3d(res_[0], res_[1], res_[2]);
        world_client = this->create_client<uvs_message::srv::UvQueryWorld>("uve_query_world");
    }
    ~GeneratorNode() = default;

    void generate()
    {
        graph_free = std::make_shared<Layer1GridGraph>(world, res);
        graph_free->save(savePath + "/" + freeMapName + ".json");

        uvs_message::srv::UvQueryWorld::Response world_carry = world;
        auto cargo = world_carry.cargos[0];
        cv::Point2f leftanchor(std::numeric_limits<double>::max(), 0);
        for (auto &p : cargo.anchors)
        {
            if (p.x < leftanchor.x)
            {
                leftanchor = cv::Point2f(p.x, p.y);
            }
        }
        cv::Point2f agentanchor(world.agents[0].anchors[0].x, world.agents[0].anchors[0].y);
        cv::Point2f offset = agentanchor - leftanchor;
        std::vector<cv::Point2f> vertices, temp;
        for (auto &p : world.agents[0].shape.points)
        {
            temp.push_back(cv::Point2f(p.x, p.y));
        }
        for (auto &p : cargo.shape.points)
        {
            temp.push_back(cv::Point2f(p.x, p.y) + offset);
        }
        cv::convexHull(temp, vertices);
        world_carry.agents[0].shape.points.clear();
        for (auto &p : vertices)
        {
            geometry_msgs::msg::Point32 point;
            point.x = p.x;
            point.y = p.y;
            world_carry.agents[0].shape.points.push_back(point);
        }
        graph_carry = std::make_shared<Layer1GridGraph>(world_carry, res);
        graph_carry->save(savePath + "/" + carryMapName + ".json");
    }
public:
    std::shared_ptr<Layer1GridGraph> graph_free;
    std::shared_ptr<Layer1GridGraph> graph_carry;
    uvs_message::srv::UvQueryWorld::Response world;
    rclcpp::Client<uvs_message::srv::UvQueryWorld>::SharedPtr world_client;

    std::string savePath;
    std::string freeMapName;
    std::string carryMapName;
    cv::Point3d res; 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeneratorNode>();
    auto request = std::make_shared<uvs_message::srv::UvQueryWorld::Request>();
    while(!node->world_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupted");
            return 0; 
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service available");

    auto result = node->world_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        node->world = *response;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "INIT DONE");

        node->generate();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GENERATE DONE");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "WORLD FAILED");
    }
    rclcpp::shutdown();
    return 0;
}