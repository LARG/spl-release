#pragma once
#include <ImageSource.hpp>
#include "json/json.h"

#include "common/srv/get_image.hpp"


// Image source that acts as a client to interface with the services in the
// image_source_emulators package.
class ExternalSource: public ImageSource
{
    rclcpp::Client<common::srv::GetImage>::SharedPtr client;
    string service_name;
    rclcpp::Publisher<msg_String>::SharedPtr publisher_;
    rclcpp::Subscription<msg_ImageSource>::SharedPtr subscription_;

    public:
    ExternalSource(string&, Json::Value&);
    void configure(Json::Value);
    void acquire_image(unordered_map<string, shared_ptr<VisionStage>>&);
    void response_callback(msg_ImageSource);
    Mat& get_image_matrix();
};