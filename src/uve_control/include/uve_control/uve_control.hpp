#ifndef UVE_CONTROL_HPP
#define UVE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>

#include "uvs_tools/mpc.hpp"
#include "uvs_message/msg/uv_emb_arm.hpp"
#include "uvs_message/msg/uv_emb_emag.hpp"
#include "uvs_message/msg/uv_emb_kinetics.hpp"

class UveControl : public rclcpp::Node
{
public:
    UveControl();
    ~UveControl();
};

#endif // UVE_CONTROL_HPP