#include "uve_perception.hpp"

#include "tf2/utils.h"

#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"

#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

UvePerception::UvePerception()
    : Node("uve_perception")
{
    loadConfig();
    init();

    pub_pose_array_ = create_publisher<uve_message::msg::UveApriltagPoseList>("uve_apriltag_pose_list", 1);
    sub_detect_ = create_subscription<std_msgs::msg::Bool>("uve_perception_detect", 1, std::bind(&UvePerception::detectCallback, this, std::placeholders::_1));
}

UvePerception::~UvePerception()
{
    if (config.apriltag.family == "tag36h11")
    {
        tag36h11_destroy(tf);
    }
    else if (config.apriltag.family == "tag25h9")
    {
        tag25h9_destroy(tf);
    }
    else if (config.apriltag.family == "tag16h5")
    {
        tag16h5_destroy(tf);
    }
    else if (config.apriltag.family == "tagCircle21h7")
    {
        tagCircle21h7_destroy(tf);
    }
    else if (config.apriltag.family == "tagCircle49h12")
    {
        tagCircle49h12_destroy(tf);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid AprilTag family");
        return;
    }
    apriltag_detector_destroy(td);
}

void UvePerception::loadConfig()
{
    declare_parameter("camera.color.stream_format", "RS2_FORMAT_BGR8");
    declare_parameter("camera.color.width", 1920);
    declare_parameter("camera.color.height", 1080);
    declare_parameter("camera.color.fps", 30);
    declare_parameter("camera.color.enable", true);
    declare_parameter("camera.depth.stream_format", "RS2_FORMAT_Z16");
    declare_parameter("camera.depth.width", 1280);
    declare_parameter("camera.depth.height", 720);
    declare_parameter("camera.depth.fps", 30);
    declare_parameter("camera.depth.enable", false);
    declare_parameter("camera.infrared.stream_format", "RS2_FORMAT_Y8");
    declare_parameter("camera.infrared.width", 1280);
    declare_parameter("camera.infrared.height", 720);
    declare_parameter("camera.infrared.fps", 30);
    declare_parameter("camera.infrared.enable", false);
    declare_parameter("apriltag.enable", true);
    declare_parameter("apriltag.family", "tag36h11");
    declare_parameter("apriltag.threads", 4);
    declare_parameter("apriltag.quad_decimate", 2.0);
    declare_parameter("apriltag.quad_sigma", 0.0);
    declare_parameter("apriltag.refine_edges", true);
    declare_parameter("apriltag.decode_sharpening", 0.25);
    declare_parameter("apriltag.debug", false);
    declare_parameter("apriltag.tagsize", 0.0752);
    declare_parameter("pubrate", 30);

    config.camera.color.stream_format = get_parameter("camera.color.stream_format").as_string();
    config.camera.color.width = get_parameter("camera.color.width").as_int();
    config.camera.color.height = get_parameter("camera.color.height").as_int();
    config.camera.color.fps = get_parameter("camera.color.fps").as_int();
    config.camera.color.enable = get_parameter("camera.color.enable").as_bool();
    config.camera.depth.stream_format = get_parameter("camera.depth.stream_format").as_string();
    config.camera.depth.width = get_parameter("camera.depth.width").as_int();
    config.camera.depth.height = get_parameter("camera.depth.height").as_int();
    config.camera.depth.fps = get_parameter("camera.depth.fps").as_int();
    config.camera.depth.enable = get_parameter("camera.depth.enable").as_bool();
    config.camera.infrared.stream_format = get_parameter("camera.infrared.stream_format").as_string();
    config.camera.infrared.width = get_parameter("camera.infrared.width").as_int();
    config.camera.infrared.height = get_parameter("camera.infrared.height").as_int();
    config.camera.infrared.fps = get_parameter("camera.infrared.fps").as_int();
    config.camera.infrared.enable = get_parameter("camera.infrared.enable").as_bool();
    config.apriltag.enable = get_parameter("apriltag.enable").as_bool();
    config.apriltag.family = get_parameter("apriltag.family").as_string();
    config.apriltag.threads = get_parameter("apriltag.threads").as_int();
    config.apriltag.quad_decimate = get_parameter("apriltag.quad_decimate").as_double();
    config.apriltag.quad_sigma = get_parameter("apriltag.quad_sigma").as_double();
    config.apriltag.refine_edges = get_parameter("apriltag.refine_edges").as_bool();
    config.apriltag.decode_sharpening = get_parameter("apriltag.decode_sharpening").as_double();
    config.apriltag.debug = get_parameter("apriltag.debug").as_bool();
    config.apriltag.tagsize = get_parameter("apriltag.tagsize").as_double();
    config.pubrate = get_parameter("pubrate").as_int();

    RCLCPP_INFO(this->get_logger(), "Camera Color Stream Format: %s", config.camera.color.stream_format.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera Color Width: %d", config.camera.color.width);
    RCLCPP_INFO(this->get_logger(), "Camera Color Height: %d", config.camera.color.height);
    RCLCPP_INFO(this->get_logger(), "Camera Color FPS: %d", config.camera.color.fps);
    RCLCPP_INFO(this->get_logger(), "Camera Color Enable: %s", config.camera.color.enable ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Camera Depth Stream Format: %s", config.camera.depth.stream_format.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera Depth Width: %d", config.camera.depth.width);
    RCLCPP_INFO(this->get_logger(), "Camera Depth Height: %d", config.camera.depth.height);
    RCLCPP_INFO(this->get_logger(), "Camera Depth FPS: %d", config.camera.depth.fps);
    RCLCPP_INFO(this->get_logger(), "Camera Depth Enable: %s", config.camera.depth.enable ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Camera Infrared Stream Format: %s", config.camera.infrared.stream_format.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera Infrared Width: %d", config.camera.infrared.width);
    RCLCPP_INFO(this->get_logger(), "Camera Infrared Height: %d", config.camera.infrared.height);
    RCLCPP_INFO(this->get_logger(), "Camera Infrared FPS: %d", config.camera.infrared.fps);
    RCLCPP_INFO(this->get_logger(), "Camera Infrared Enable: %s", config.camera.infrared.enable ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "AprilTag Enable: %s", config.apriltag.enable ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "AprilTag Family: %s", config.apriltag.family.c_str());
    RCLCPP_INFO(this->get_logger(), "AprilTag Threads: %d", config.apriltag.threads);
    RCLCPP_INFO(this->get_logger(), "AprilTag Quad Decimate: %f", config.apriltag.quad_decimate);
    RCLCPP_INFO(this->get_logger(), "AprilTag Quad Sigma: %f", config.apriltag.quad_sigma);
    RCLCPP_INFO(this->get_logger(), "AprilTag Refine Edges: %s", config.apriltag.refine_edges ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "AprilTag Decode Sharpening: %f", config.apriltag.decode_sharpening);
    RCLCPP_INFO(this->get_logger(), "AprilTag Debug: %s", config.apriltag.debug ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "AprilTag Tag Size: %f", config.apriltag.tagsize);
    RCLCPP_INFO(this->get_logger(), "Publish Rate: %d", config.pubrate);
}

void UvePerception::init()
{
    rs2::config cfg;
    if (config.camera.color.enable)
    {
        cfg.enable_stream(RS2_STREAM_COLOR, config.camera.color.width, config.camera.color.height, RS2_FORMAT_BGR8, config.camera.color.fps);
    }
    if (config.camera.depth.enable)
    {
        cfg.enable_stream(RS2_STREAM_DEPTH, config.camera.depth.width, config.camera.depth.height, RS2_FORMAT_Z16, config.camera.depth.fps);
    }
    if (config.camera.infrared.enable)
    {
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, config.camera.infrared.width, config.camera.infrared.height, RS2_FORMAT_Y8, config.camera.infrared.fps);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, config.camera.infrared.width, config.camera.infrared.height, RS2_FORMAT_Y8, config.camera.infrared.fps);
    }
    pipe.start(cfg);
    for (int i = 0; i < 30; i++)
    {
        frames = pipe.wait_for_frames();
    }

    if (config.apriltag.enable)
    {
        if (config.apriltag.family == "tag36h11")
        {
            tf = tag36h11_create();
        }
        else if (config.apriltag.family == "tag25h9")
        {
            tf = tag25h9_create();
        }
        else if (config.apriltag.family == "tag16h5")
        {
            tf = tag16h5_create();
        }
        else if (config.apriltag.family == "tagCircle21h7")
        {
            tf = tagCircle21h7_create();
        }
        else if (config.apriltag.family == "tagCircle49h12")
        {
            tf = tagCircle49h12_create();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid AprilTag family");
            return;
        }
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
        td->quad_decimate = config.apriltag.quad_decimate;
        td->quad_sigma = config.apriltag.quad_sigma;
        td->refine_edges = config.apriltag.refine_edges;
        td->decode_sharpening = config.apriltag.decode_sharpening;
        td->debug = config.apriltag.debug;
    }

    rs2::video_stream_profile color_profile = frames.get_color_frame().get_profile().as<rs2::video_stream_profile>();
    auto color_intrinsics = color_profile.get_intrinsics();
    camera_matrix = cv::Mat(cv::Size(3, 3), CV_64F);
    camera_matrix.at<double>(0, 0) = color_intrinsics.fx;
    camera_matrix.at<double>(0, 1) = 0;
    camera_matrix.at<double>(0, 2) = color_intrinsics.ppx;
    camera_matrix.at<double>(1, 0) = 0;
    camera_matrix.at<double>(1, 1) = color_intrinsics.fy;
    camera_matrix.at<double>(1, 2) = color_intrinsics.ppy;
    camera_matrix.at<double>(2, 0) = 0;
    camera_matrix.at<double>(2, 1) = 0;
    camera_matrix.at<double>(2, 2) = 1;
    
    RCLCPP_INFO(get_logger(), "init done");
}

void UvePerception::detectCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data == detect_.data)
    {
        return;
    }
    detect_.data = msg->data;
    if (detect_.data)
    {
        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / config.pubrate), std::bind(&UvePerception::timerCallback, this));
    }
    else
    {
        timer_.reset();
    }
}

void UvePerception::timerCallback()
{
    frames = pipe.wait_for_frames();
    rs2::video_frame color_frame = frames.get_color_frame();
    cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color, color, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(color, color);
    image_u8_t im = {color.cols, color.rows, color.cols, color.data};
    zarray_t *detections = apriltag_detector_detect(td, &im);
    uve_message::msg::UveApriltagPoseList pose_list;
    for (int i = 0; i < zarray_size(detections); i++)
    {
        // RCLCPP_INFO(get_logger(),"1");
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        // RCLCPP_INFO(get_logger(),"2");
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = config.apriltag.tagsize;
        // RCLCPP_INFO(get_logger(),"%d, %d", camera_matrix.size().height, camera_matrix.size().width);
        info.fx = camera_matrix.at<double>(0, 0);
        info.fy = camera_matrix.at<double>(1, 1);
        info.cx = camera_matrix.at<double>(0, 2);
        info.cy = camera_matrix.at<double>(1, 2);
        // RCLCPP_INFO(get_logger(),"3");
        apriltag_pose_t pose;
        estimate_tag_pose(&info, &pose);
        // RCLCPP_INFO(get_logger(),"4");
        uve_message::msg::UveApriltagPose pose_msg;
        pose_msg.code = det->id;
        pose_msg.relative_pose.position.x = pose.t->data[0];
        pose_msg.relative_pose.position.y = pose.t->data[1];
        pose_msg.relative_pose.position.z = pose.t->data[2];
        tf2::Matrix3x3 rot_mat(
            pose.R->data[0], pose.R->data[1], pose.R->data[2],
            pose.R->data[3], pose.R->data[4], pose.R->data[5],
            pose.R->data[6], pose.R->data[7], pose.R->data[8]
        );
        tf2::Quaternion q;
        rot_mat.getRotation(q);
        pose_msg.relative_pose.orientation.x = q.x();
        pose_msg.relative_pose.orientation.y = q.y();
        pose_msg.relative_pose.orientation.z = q.z();
        pose_msg.relative_pose.orientation.w = q.w();
        pose_list.list.push_back(pose_msg);
    }
    pub_pose_array_->publish(pose_list);
    RCLCPP_INFO(get_logger(), "Published %d AprilTag poses", pose_list.list.size());
}
