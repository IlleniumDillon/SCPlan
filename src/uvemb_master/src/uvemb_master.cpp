#include "uvemb_master.hpp"

#include <thread>
#include <future>
#include <chrono>

#include <sys/unistd.h>

#include "uvemb_usb.hpp"

UvEmbMaster::UvEmbMaster()
    : Node("uvemb_master")
{
    drv = std::make_shared<UvEmbUsb>();
    while(!drv->init())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    pubStatus = this->create_publisher<uvemb_message::msg::UvEmbStatus>
        ("uvemb_status", 1);
    subArm = this->create_subscription<uvemb_message::msg::UvEmbArm>
        ("uvemb_arm", 1, std::bind(&UvEmbMaster::armCallback, this, std::placeholders::_1));
    subEmag = this->create_subscription<uvemb_message::msg::UvEmbEmag>
        ("uvemb_emag", 1, std::bind(&UvEmbMaster::emagCallback, this, std::placeholders::_1));
    subKinetics = this->create_subscription<uvemb_message::msg::UvEmbKinetics>
        ("uvemb_kinetics", 1, std::bind(&UvEmbMaster::kineticsCallback, this, std::placeholders::_1));
    upFlag = true;
    fut = std::async(std::launch::async, &UvEmbMaster::spin, this, std::ref(upFlag));
}

UvEmbMaster::~UvEmbMaster()
{
    upFlag = false;
    fut.wait();
}

void UvEmbMaster::armCallback(const uvemb_message::msg::UvEmbArm::SharedPtr msg)
{
    // drv->write_regCh((void*)msg.get(), sizeof(uvemb_message::msg::UvEmbArm));
}

void UvEmbMaster::emagCallback(const uvemb_message::msg::UvEmbEmag::SharedPtr msg)
{
    // drv->write_regCh((void*)msg.get(), sizeof(uvemb_message::msg::UvEmbArm));
}

void UvEmbMaster::kineticsCallback(const uvemb_message::msg::UvEmbKinetics::SharedPtr msg)
{
    // drv->write_regCh((void*)msg.get(), sizeof(uvemb_message::msg::UvEmbArm));
}

void UvEmbMaster::spin(bool& run)
{
    while (run)
    {
        RCLCPP_INFO(get_logger(), "Spin");
        uvemb_message::msg::UvEmbStatus msg;
        pubStatus->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
