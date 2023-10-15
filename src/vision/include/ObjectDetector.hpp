#pragma once
#include <boilerplate.hpp>
#include <VisionStage.hpp>
#include <ImageSource.hpp>

#define MAX_OBJECT_CLASSES 20

enum object_attribs{
    OBJ_TYPE,
    CENTER_X,
    CENTER_Y,
    WIDTH_X,
    WIDTH_Y,
    CONFIDENCE
}; 

#ifndef OBJ_NAMES
#define OBJ_NAMES
static map<string, int> object_names_map {
    {"ball", IO_BALL},
    {"cross", IO_CROSS},
    {"robot", IO_ROBOT}
};
#endif

class ObjectDetector: public VisionStage
{
    protected:
    rclcpp::Publisher<msg_DetectedObjects>::SharedPtr objects_publisher; // TODO: Custom message type
    unsigned int frame_count=0;
    string image_source_name;
    vector<int> object_map;

    public:
    ObjectDetector(string name_, Json::Value config);
    void execute(unordered_map<string, shared_ptr<VisionStage>>&);
    virtual vector<msg_ImageObject> detect(unordered_map<string, shared_ptr<VisionStage>>&) = 0;
    virtual void configure(Json::Value);
};
