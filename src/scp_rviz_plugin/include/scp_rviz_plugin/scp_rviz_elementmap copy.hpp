#ifndef SCP_RVIZ_PLUGIN__SCP_RVIZ_ELEMENTMAP_HPP_
#define SCP_RVIZ_PLUGIN__SCP_RVIZ_ELEMENTMAP_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/msg_conversions.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include "rviz_common/message_filter_display.hpp"
#include <rviz_default_plugins/displays/image/ros_image_texture.hpp>
#include <scp_message/msg/model_state_list.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "rviz_default_plugins/displays/map/map_display.hpp"
#include "rviz_default_plugins/displays/map/palette_builder.hpp"
#include "rviz_default_plugins/displays/map/swatch.hpp"

#include "Map.hpp"

using namespace rviz_default_plugins::displays;

namespace scp_rviz_plugin
{

class SCElementMap : public rviz_common::MessageFilterDisplay<scp_message::msg::ModelStateList>
{
    Q_OBJECT
public:
    explicit SCElementMap(rviz_common::DisplayContext * context);
    SCElementMap();
    ~SCElementMap() override;

    void onInitialize() override;
    void fixedFrameChanged() override;
    void reset() override;

    void processMessage(scp_message::msg::ModelStateList::ConstSharedPtr msg) override;

    bool loadModelTemplate(std::string modelName, Element& element);

protected Q_SLOTS:
    void update();
Q_SIGNALS:
    void newMap();

private Q_SLOTS:
    void updateResolution();
    void updateWidth();
    void updateHeight();

private:
    float resolution, width, height;

    scp_message::msg::ModelStateList msg;

    rviz_common::properties::FloatProperty * resolutionProperty;
    rviz_common::properties::FloatProperty * widthProperty;
    rviz_common::properties::FloatProperty * heighPproperty;

    std::string modelPath;

    Element agentTemplate;
    Element goodTemplate;
    Element obstacleTemplate;
    Element wall10Template;
    Element wall12Template;

    ElementMap elementMap;

    cv::Mat map;


    std::vector<std::shared_ptr<Swatch>> swatches_;
    std::vector<Ogre::TexturePtr> palette_textures_;
    std::vector<bool> color_scheme_transparency_;
    bool loaded_;
};

} // namespace scp_rviz_plugin

#endif // SCP_RVIZ_PLUGIN__SCP_RVIZ_ELEMENTMAP_HPP_