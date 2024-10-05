#ifndef UVEMB_MASTER_HPP
#define UVEMB_MASTER_HPP

#include <rclcpp/rclcpp.hpp>

#include <iostream>

#include "uvemb_usb.hpp"
#include "uvemb_message/msg/uv_emb_arm.hpp"
#include "uvemb_message/msg/uv_emb_emag.hpp"
#include "uvemb_message/msg/uv_emb_kinetics.hpp"
#include "uvemb_message/msg/uv_emb_status.hpp"

class UvEmbMaster : public rclcpp::Node
{
public:
    UvEmbMaster();
    ~UvEmbMaster();
private:
    rclcpp::Publisher<uvemb_message::msg::UvEmbStatus>::SharedPtr pubStatus;
    rclcpp::Subscription<uvemb_message::msg::UvEmbArm>::SharedPtr subArm;
    rclcpp::Subscription<uvemb_message::msg::UvEmbEmag>::SharedPtr subEmag;
    rclcpp::Subscription<uvemb_message::msg::UvEmbKinetics>::SharedPtr subKinetics;

    void armCallback(const uvemb_message::msg::UvEmbArm::SharedPtr msg);
    void emagCallback(const uvemb_message::msg::UvEmbEmag::SharedPtr msg);
    void kineticsCallback(const uvemb_message::msg::UvEmbKinetics::SharedPtr msg);

    void spin(bool& run);

    std::shared_ptr<UvEmbUsb> drv;
    std::future<void> fut;
    bool upFlag = false;
};

#endif // UVEMB_MASTER_HPP