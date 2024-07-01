#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_AGENTACTION_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_AGENTACTION_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosAgentActionPrivate;

class GazeboRosAgentAction : public gazebo::WorldPlugin
{
public:
    /// Constructor
    GazeboRosAgentAction();

    /// Destructor
    ~GazeboRosAgentAction();

protected:
    // Documentation inherited
    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
    /// Private data pointer
    std::unique_ptr<GazeboRosAgentActionPrivate> impl_;
};
}

#endif // GAZEBO_PLUGINS__GAZEBO_ROS_AGENTACTION_HPP_