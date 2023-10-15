#pragma once
#include <boilerplate.hpp>
#include <VisionStage.hpp>
#include <ImageSource.hpp>


class ColorSegmenter : public ImageSource
{
    private:
    Mat segmented_image, rgb_image;
    string raw_image_source;
    int bilateral_kernel_size = 5;
    unordered_map<string, vector<int>> color_ranges;
    int input_subsample_major = 1;
    int input_subsample_minor = 1;

    public:
    ColorSegmenter(string&, Json::Value&);
    void acquire_image(unordered_map<string, shared_ptr<VisionStage>>&);
    Mat& get_image_matrix();
    Mat& get_segmented_image();
    void configure(Json::Value);
};