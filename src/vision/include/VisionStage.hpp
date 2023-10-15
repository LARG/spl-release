#pragma once
#include <boilerplate.hpp>
#include <opencv.hpp>
#include "json/json.h"

using namespace std;
using namespace cv;


#define ReadJSONAttribute(cfg, attr, typ) {attr = cfg["" #attr ""].as##typ();}
#define ReadJSONAttributeIfExists(cfg, attr, typ) {if(cfg.isMember("" #attr "")) attr = cfg["" #attr ""].as##typ();}


void updateJsonValue(Json::Value&, const Json::Value&);


class VisionStage : public rclcpp::Node
{
    public:
    string stage_name, stage_type;
    Json::Value cfg, invoke_cfg;
    VisionStage(string, Json::Value);
    virtual void execute(unordered_map<string, shared_ptr<VisionStage>>&) = 0;
    virtual void configure(Json::Value)=0;
    void reconfigure(Json::Value);
};