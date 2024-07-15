#include "_pose_tool2.hpp"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace scp_rviz_plugin
{
PoseTool2::PoseTool2()
: rviz_common::Tool(), arrow_(nullptr), angle_(0)
{
  RVIZ_COMMON_LOG_INFO_STREAM("PoseTool2 constructor");
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

PoseTool2::~PoseTool2() = default;

void PoseTool2::onInitialize()
{
  arrow_ = std::make_shared<rviz_rendering::Arrow>(
    scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void PoseTool2::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = Position;
}

void PoseTool2::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

int PoseTool2::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

  if (event.leftDown()) {
    //RVIZ_COMMON_LOG_INFO_STREAM("Left mouse button pressed");
    return processMouseLeftButtonPressed(point_projection_on_xy_plane);
  } else if (event.type == QEvent::MouseMove && event.left()) {
    //RVIZ_COMMON_LOG_INFO_STREAM("Mouse moved");
    return processMouseMoved(point_projection_on_xy_plane);
  } else if (event.leftUp()) {
    //RVIZ_COMMON_LOG_INFO_STREAM("Left mouse button released");
    return processMouseLeftButtonReleased();
  }

  return 0;
}

int PoseTool2::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
  int flags = 0;
  assert(state_ == Position);
  if (xy_plane_intersection.first) {
    arrow_position_ = xy_plane_intersection.second;
    arrow_->setPosition(arrow_position_);

    state_ = Orientation;
    flags |= Render;
  }
  return flags;
}

int PoseTool2::processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
  int flags = 0;
  if (state_ == Orientation) {
    // compute angle in x-y plane
    if (xy_plane_intersection.first) {
      angle_ = calculateAngle(xy_plane_intersection.second, arrow_position_);
      makeArrowVisibleAndSetOrientation(angle_);

      flags |= Render;
    }
  }

  return flags;
}

void PoseTool2::makeArrowVisibleAndSetOrientation(double angle)
{
  arrow_->getSceneNode()->setVisible(true);

  // we need base_orient, since the arrow goes along the -z axis by default
  // (for historical reasons)
  Ogre::Quaternion orient_x = Ogre::Quaternion(
    Ogre::Radian(-Ogre::Math::HALF_PI),
    Ogre::Vector3::UNIT_Y);

  arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
}

int PoseTool2::processMouseLeftButtonReleased()
{
  int flags = 0;
  if (state_ == Orientation) {
    flags |= onPoseSet2(arrow_position_.x, arrow_position_.y, angle_);
    // flags |= (Finished | Render);
    state_ = Position;
  }

  return flags;
}

double PoseTool2::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point)
{
  return atan2(start_point.y - end_point.y, start_point.x - end_point.x);
}

geometry_msgs::msg::Quaternion PoseTool2::orientationAroundZAxis(double angle)
{
  auto orientation = geometry_msgs::msg::Quaternion();
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = sin(angle) / (2 * cos(angle / 2));
  orientation.w = cos(angle / 2);
  return orientation;
}

void PoseTool2::logPose(
  std::string designation, geometry_msgs::msg::Point position,
  geometry_msgs::msg::Quaternion orientation, double angle, std::string frame)
{
  RVIZ_COMMON_LOG_INFO_STREAM(
    "Setting " << designation << " pose: Frame:" << frame << ", Position(" << position.x << ", " <<
      position.y << ", " << position.z << "), Orientation(" << orientation.x << ", " <<
      orientation.y << ", " << orientation.z << ", " << orientation.w <<
      ") = Angle: " << angle);
}
}  // namespace scp_rviz_plugin