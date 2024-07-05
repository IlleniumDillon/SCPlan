#include "scp_rviz_elementmap.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include "rclcpp/time.hpp"

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/objects/grid.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_default_plugins/displays/map/palette_builder.hpp"

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tf2/utils.h"

#include "json/json.h"

namespace scp_rviz_plugin
{
SCElementMap::SCElementMap()
    : rviz_common::MessageFilterDisplay<scp_message::msg::ModelStateList>(),
      resolution(0.1), width(0.0), height(0.0)
{
    try
    {
        modelPath = ament_index_cpp::get_package_share_directory("scp_simulate")
            + "/models/";
    }
    catch(ament_index_cpp::PackageNotFoundError& e)
    {
        setStatus(rviz_common::properties::StatusProperty::Error, "file", e.what());
    }

    loadModelTemplate("agent", agentTemplate);
    loadModelTemplate("good", goodTemplate);
    loadModelTemplate("obstacle", obstacleTemplate);
    loadModelTemplate("wall10", wall10Template);
    loadModelTemplate("wall12", wall12Template);

    connect(this, SIGNAL(newMap()), this, SLOT(update()));

    resolutionProperty = new rviz_common::properties::FloatProperty(
        "Resolution", 0.1, "Resolution of the map", this, SLOT(updateResolution())
    );
    resolutionProperty->setMax(1.0);
    resolutionProperty->setMin(0.01);
    widthProperty = new rviz_common::properties::FloatProperty(
        "Width", 0.0, "Width of the map", this, SLOT(updateWidth())
    );
    widthProperty->setMax(1000.0);
    widthProperty->setMin(0.0);
    heighPproperty = new rviz_common::properties::FloatProperty(
        "Height", 0.0, "Height of the map", this, SLOT(updateHeight())
    );
    heighPproperty->setMax(1000.0);
    heighPproperty->setMin(0.0);
}
SCElementMap::SCElementMap(rviz_common::DisplayContext *context)
    : SCElementMap()
{
    context_ = context;
    scene_manager_ = context->getSceneManager();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

SCElementMap::~SCElementMap()
{
    unsubscribe();
    // clear();
}

void SCElementMap::onInitialize()
{
    MFDClass::onInitialize();
    rviz_ros_node_ = context_->getRosNodeAbstraction();
}
void SCElementMap::fixedFrameChanged()
{
}
void SCElementMap::reset()
{
}
void SCElementMap::processMessage(scp_message::msg::ModelStateList::ConstSharedPtr msg)
{
    this->msg = *msg;
    elementMap.elements.clear();
    for (int i = 0; i < msg->modelstates.size(); i++)
    {
        Element element;
        std::string type = msg->modelstates[i].name.substr(0, msg->modelstates[i].name.find("_"));
        if (type == "good")
        {
            element.id = 0;
            element.originShape = goodTemplate.originShape;
            element.originAnchors = goodTemplate.originAnchors;
        }
        else if (type == "obstacle")
        {
            element.id = 1;
            element.originShape = obstacleTemplate.originShape;
            element.originAnchors = obstacleTemplate.originAnchors;
        }
        else if (type == "wall10")
        {
            element.id = 2;
            element.originShape = wall10Template.originShape;
            element.originAnchors = wall10Template.originAnchors;
        }
        else if (type == "wall12")
        {
            element.id = 2;
            element.originShape = wall12Template.originShape;
            element.originAnchors = wall12Template.originAnchors;
        }
        else if (type == "agent")
        {
            element.id = 3;
            element.originShape = agentTemplate.originShape;
            element.originAnchors = agentTemplate.originAnchors;
        }
        else
        {
            continue;
        }
        element.name = msg->modelstates[i].name;
        CoordD panning(
            msg->modelstates[i].pose.position.x,
            msg->modelstates[i].pose.position.y);
        double rotation = tf2::getYaw(msg->modelstates[i].pose.orientation);
        element.setGeometry(panning, rotation);
        elementMap.elements.push_back(element);
    }
    emit newMap();
}

bool SCElementMap::loadModelTemplate(std::string modelName, Element &element)
{
    std::string modelFile = modelPath + modelName + ".json";
    std::ifstream file(modelFile);
    if (!file.is_open())
    {
        return false;
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(file, root))
    {
        file.close();
        return false;
    }
    file.close();

    if (root["isAgent"].isNull() ||
        root["isStatic"].isNull() ||
        root["originShape"].isNull())
    {
        return false;
    }

    element.isAgent = root["isAgent"].asBool();
    element.isStatic = root["isStatic"].asBool();

    Json::Value originShape = root["originShape"];
    std::vector<CoordD> vertices;
    for (int i = 0; i < originShape.size(); i++)
    {
        CoordD vertex(
            originShape[i][0].asDouble(),
            originShape[i][1].asDouble());
        vertices.push_back(vertex);
    }
    element.originShape = Polygon(vertices);
    element.id = -1;

    if (!root["originAnchors"].isNull())
    {
        Json::Value originAnchors = root["originAnchors"];
        std::vector<CoordD> anchors;
        for (int i = 0; i < originAnchors.size(); i++)
        {
            CoordD anchor(
                originAnchors[i][0].asDouble(),
                originAnchors[i][1].asDouble());
            anchors.push_back(anchor);
        }
        element.originAnchors = anchors;
    }

    return true;
}

void SCElementMap::updateResolution()
{
    resolution = resolutionProperty->getFloat();
}

void SCElementMap::updateWidth()
{
    width = widthProperty->getFloat();
}

void SCElementMap::updateHeight()
{
    height = heighPproperty->getFloat();
}

void SCElementMap::update()
{
    map = cv::Mat(height / resolution, width / resolution, CV_8UC3, cv::Scalar(255, 255, 255));

    for (Element &e : elementMap.elements)
    {
        cv::Scalar color;
        switch (e.id)
        {
        case 0:
            color = cv::Scalar(0, 255, 0);
            break;
        case 1:
            color = cv::Scalar(128, 128, 128);
            break;
        case 2:
            color = cv::Scalar(0, 0, 0);
            break;
        case 3:
            color = cv::Scalar(255, 255, 0);
            break;
        }
        std::vector<cv::Point> points;
        for (CoordD &p : e.shape.vertices)
        {
            points.push_back(cv::Point(
                (width / 2 + p.x) / resolution,
                (height / 2 - p.y) / resolution
            ));
        }
        cv::fillConvexPoly(map, points.data(), points.size(), color);
    }

    cv::imshow("Map", map);
    cv::waitKey(1);
}

} // namespace scp_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(scp_rviz_plugin::SCElementMap, rviz_common::Display)


