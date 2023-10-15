#pragma once
#include <boilerplate.hpp>
#include "json/json.h"

#include <VisionStage.hpp>

#include <ImageSource.hpp>
#include <CameraCapture.hpp>
#include <StaticImage.hpp>
#include <ExternalSource.hpp>

#include <ObjectDetector.hpp>
#include <TFLiteDetector.hpp>

#include <LineDetector.hpp>
#include <HoughLineDetector.hpp>

#include <ColorSegmenter.hpp>


using namespace std;
using namespace cv;

#define VISION_NODE_CONFIG_PATH "./configs/vision/VisionNode.json"

class VisionNode : public rclcpp::Node
{
    public:
    VisionNode();
    void register_stages(rclcpp::executors::MultiThreadedExecutor&);
    
    
    private:
    Json::Value default_config;
    rclcpp::Subscription<msg_String>::SharedPtr invoke_subscription;
    rclcpp::Subscription<msg_String>::SharedPtr reconfigure_subscription;
    unordered_map<string, shared_ptr<VisionStage>> vision_stages;

    void invoke_callback(msg_String);
    void reconfigure_callback(msg_String);
};