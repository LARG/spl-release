#include <ColorSegmenter.hpp>

ColorSegmenter::ColorSegmenter(string& name_, Json::Value& config) : ImageSource(name_, config)
{
    configure(config);
}


void ColorSegmenter::configure(Json::Value config)
{
    ImageSource::configure(config);
    segmented_image.create(image_height, image_width, CV_8UC3);
    ReadJSONAttribute(config,raw_image_source,String);

    // Read color thresholding ranges
    auto color_configs = config["color_ranges"];
    for(auto it = color_configs.begin(); it != color_configs.end(); it++)
    {
        string color = it.key().asString();
        vector<int> color_range;
        for(auto comp: *it)
            color_range.push_back(comp.asInt());
        color_ranges[color] = color_range;
    }

    ReadJSONAttributeIfExists(config,input_subsample_major,Int);
    ReadJSONAttributeIfExists(config,input_subsample_minor,Int);
}


void ColorSegmenter::acquire_image(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
    auto image_source = dynamic_pointer_cast<ImageSource>(stages[raw_image_source]);
    struct CopyConfig copy_cfg;
    copy_cfg.subsample_major = input_subsample_major;
    copy_cfg.subsample_minor = input_subsample_minor;
    auto image_mat = image_source->copy_to_mat(copy_cfg);

    Mat reduced_image, blur, hsv_image, out_image, field_mask, line_mask;
    // resize(image_mat, reduced_image, Size(256, 192), 0, 0, INTER_NEAREST);

    cvtColor(image_mat, rgb_image, COLOR_YUV2BGR_YUYV);

    cvtColor(rgb_image, hsv_image, COLOR_BGR2HSV);

    auto field_range = color_ranges["IP_FIELD"];

    // Threshold in HSV space to get a mask of field pixels
    inRange(hsv_image,
            Scalar(field_range[0], field_range[1], field_range[2]),
            Scalar(field_range[3], field_range[4], field_range[5]),
            field_mask
    );

    Mat field_mask_blur, inside_field_mask, non_field_mask;
    
    // Blur the field mask so it includes pixels close to the field pixels.
    cv::blur(field_mask, field_mask_blur, Size(5,5));
    inRange(field_mask_blur, Scalar(100), Scalar(255), inside_field_mask);

    // Get a mask of the pixels that are not on the field but are close to the field
    bitwise_xor(field_mask, inside_field_mask, non_field_mask, inside_field_mask);

    // Get the segmented image suitable for line detection by masking out all the field pixels
    segmented_image = Mat::zeros( segmented_image.size(), segmented_image.type() );
    bitwise_and(rgb_image, rgb_image, segmented_image, non_field_mask);
}

Mat& ColorSegmenter::get_image_matrix()
{
    return segmented_image;
}
