#pragma once
#include <ImageSource.hpp>
#include "json/json.h"

class CameraCapture: public ImageSource
{
    VideoCapture cv_capture;

    public:
    CameraCapture(string&, Json::Value&);
    void acquire_image(unordered_map<string, shared_ptr<VisionStage>>&);
    void configure(Json::Value);
    Mat& get_image_matrix();
};