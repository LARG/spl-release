#pragma once
#include <boilerplate.hpp>
#include <VisionStage.hpp>
#include <ImageSource.hpp>


class LineDetector: public VisionStage
{
    protected:
    rclcpp::Publisher<msg_DetectedLines>::SharedPtr lines_publisher;
    unsigned int frame_count=0;
    string image_source_name;

    public:
    LineDetector(string& name_, Json::Value&);
    void execute(unordered_map<string, shared_ptr<VisionStage>>&);
    virtual vector<msg_ImageLine> detect(unordered_map<string, shared_ptr<VisionStage>>&) = 0;
    virtual void configure(Json::Value);
};