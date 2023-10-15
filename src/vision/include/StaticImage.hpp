#pragma once
#include <ImageSource.hpp>
#include "json/json.h"


class StaticImage : public ImageSource
{
    public:
    StaticImage(string&, Json::Value&);
    void acquire_image(unordered_map<string, shared_ptr<VisionStage>>&);
    Mat& get_image_matrix();
    void configure(Json::Value);
};
