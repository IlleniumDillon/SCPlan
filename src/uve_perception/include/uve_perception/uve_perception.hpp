#ifndef UVE_PERCEPTION_HPP
#define UVE_PERCEPTION_HPP

#include <rclcpp/rclcpp.hpp>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "apriltag.h"

#include "uve_message/msg/uve_apriltag_pose_list.hpp"
#include <std_msgs/msg/bool.hpp>

typedef struct config
{
    struct
    {
        struct
        {
            std::string stream_format;
            int width;
            int height;
            int fps;
            bool enable;
        }color;
        struct
        {
            std::string stream_format;
            int width;
            int height;
            int fps;
            bool enable;
        }depth;
        struct
        {
            std::string stream_format;
            int width;
            int height;
            int fps;
            bool enable;
        }infrared;
    }camera;

    struct
    {
        bool enable;
        std::string family;
        int threads;
        float quad_decimate;
        float quad_sigma;
        bool refine_edges;
        double decode_sharpening;
        bool debug;
        double tagsize;
    }apriltag;

    int pubrate;

}perceptionConfig;

class UvePerception : public rclcpp::Node
{
public:
    UvePerception();
    ~UvePerception();
private:
    void loadConfig();
    void init();
    void detectCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void timerCallback();
private:
    perceptionConfig config;
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    rs2::pipeline pipe;
    rs2::frameset frames;
    cv::Mat camera_matrix;

    rclcpp::Publisher<uve_message::msg::UveApriltagPoseList>::SharedPtr pub_pose_array_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_detect_;
    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Bool detect_;

};

#endif  // UVE_PERCEPTION_HPP