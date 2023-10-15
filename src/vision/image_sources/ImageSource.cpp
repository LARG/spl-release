#include <ImageSource.hpp>


float CopyConfig::get_scale_factor()
{
    return ((float)(subsample_major + subsample_minor))/2;
}


ImageSource::ImageSource(string& name_, Json::Value config) : VisionStage(name_, config)
{
    configure(config);
    publisher_ = this->create_publisher<msg_ImageSource>(TOPIC_VISION_PUBLISHED_IMAGES, 10);
}


void ImageSource::configure(Json::Value config)
{
    ReadJSONAttribute(config, image_width, Int);
    ReadJSONAttribute(config, image_height, Int);
    ReadJSONAttribute(config, color_space, String);
    
    if (color_space == "YUYV")
        channels = 4;
    else if (color_space == "RGB")
        channels = 3;
    else
    {
        RCLCPP_ERROR(this->get_logger(), "ImageSource %s unsupported color space.", stage_name.c_str(), color_space.c_str());
        exit(1);
    }

    ReadJSONAttributeIfExists(config,flip_horizontal,Bool);
    ReadJSONAttributeIfExists(config,flip_vertical,Bool);

    if(config.isMember("publish"))
    {
        ReadJSONAttribute(config,publish,Bool);
        // Publish every "publish_frame_interval" frame
        ReadJSONAttributeIfExists(config,publish_frame_interval,Int);
        // Subsample to save network bandwidth
        ReadJSONAttributeIfExists(config,publish_subsample_major,Int);
        ReadJSONAttributeIfExists(config,publish_subsample_minor,Int);
    }
}


void ImageSource::execute(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
    // Acquire image
    acquire_image(stages);
    std::chrono::high_resolution_clock::time_point frame_acquire_time = std::chrono::high_resolution_clock::now();
    long int timestamp = std::chrono::duration_cast<std::chrono::seconds>(frame_acquire_time.time_since_epoch()).count();
    RCLCPP_DEBUG(this->get_logger(), "ImageSource %s acquired image at %ul s", stage_name.c_str(),
                timestamp);

    // Publish if necessary
    if(publish && (frame_count%publish_frame_interval == 0))
    {
        auto pub_thr = thread( [&] {publish_image(timestamp);});
        pub_thr.detach();
        RCLCPP_DEBUG(this->get_logger(), "ImageSource %s published image.", stage_name.c_str());
    }

    frame_count++;
}

Vec2i ImageSource::get_image_size()
{
    auto image = get_image_matrix();
    return Vec2f(image.cols, image.rows);
}


void ImageSource::update_copy_config_roi(struct CopyConfig &copy_config, Vec4f roi)
{
    auto image = get_image_matrix();
    Vec4i roi_pixels(roi.mul(
        Vec4f(image.cols, image.rows, image.cols, image.rows)
        ) - Vec4f(0, 0, 1, 1)
    );

    copy_config.input_channels = channels;

    for(int i=0; i<4; i++)
    {
        if(copy_config.rect[i]<0 || copy_config.rect[i] >= ((i%2)?image.rows:image.cols))
        {
            RCLCPP_ERROR(this->get_logger(),
            "ImageSource %s: ROI [%d , %d , %d , %d] "
            "from normalized coordinates [%.2f , %.2f , %.2f , %.2f] "
            "is out of bounds for image size %d x %d",
            stage_name.c_str(),
            roi_pixels[0], roi_pixels[1], roi_pixels[2], roi_pixels[3],
            roi[0], roi[1], roi[2], roi[3],
            image.cols, image.rows
            );
            exit(1);
        }
        copy_config.rect[i] = roi_pixels[i];
    }
}

void ImageSource::update_copy_config_roi(struct CopyConfig &copy_config, Vec2f center, Vec2i copy_size)
{
    auto image = get_image_matrix();
    int subsample_total = copy_config.subsample_major + copy_config.subsample_minor;

    // Get the size of the input region copied in normalized units
    Vec2f input_roi_dims_norm(Vec2f(1.0/image.cols, 1.0/image.rows).mul(copy_size) * copy_config.get_scale_factor());
    // RCLCPP_INFO(this->get_logger(), "Input roi dims norm: %f %f", input_roi_dims_norm[0], input_roi_dims_norm[1]);

    if(input_roi_dims_norm[0]>1 || input_roi_dims_norm[1]>1)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "ImageSource %s: scaled copy size %f x %f is bigger than image size.",
            stage_name.c_str(),
            input_roi_dims_norm[0], input_roi_dims_norm[1]
            );
        exit(1);
    }
    
    // Get a suitable copied region within bounds that contains the given center point
    Vec2f top_left = center - input_roi_dims_norm/2.0;

    Vec2i top_left_px(top_left.mul(Vec2f(image.cols, image.rows)));
    Vec2f shift_top_left_px = Vec2f(
        (top_left[0]<0)?-top_left_px[0]:0,
        (top_left[1]<0)?-top_left_px[1]:0
        );
    top_left_px += shift_top_left_px;

    Vec2i input_roi_dims_px = copy_size / 2 * subsample_total;

    Vec2i bottom_right_px = top_left_px + input_roi_dims_px - Vec2i(1, 1);

    Vec2i shift_bottom_right_px = Vec2f(
        (bottom_right_px[0]>=image.cols)?image.cols-1-bottom_right_px[0]:0,
        (bottom_right_px[1]>=image.rows)?image.rows-1-bottom_right_px[1]:0
        );
    bottom_right_px += shift_bottom_right_px; top_left_px += shift_bottom_right_px;

    // Update the copy config with the pixel roi found above
    copy_config.rect[0] = top_left_px[0];
    copy_config.rect[1] = top_left_px[1];
    copy_config.rect[2] = bottom_right_px[0];
    copy_config.rect[3] = bottom_right_px[1];

    copy_config.input_channels = channels;
}


Mat ImageSource::copy_to_mat(struct CopyConfig& copy_cfg)
{
    float scale_factor = copy_cfg.get_scale_factor();

    unsigned int rmin = copy_cfg.rect[1], cmin = copy_cfg.rect[0];
    unsigned int rmax = (copy_cfg.rect[3]==-1)?image_height-1:copy_cfg.rect[3];
    unsigned int cmax = (copy_cfg.rect[2]==-1)?image_width-1:copy_cfg.rect[2];
    if (color_space=="YUYV")
    {
        cmin -= cmin%2;
        cmax -= (1-cmax%2);
    }

    unsigned int subsample_total = copy_cfg.subsample_major + copy_cfg.subsample_minor;
    int copy_width = ((cmax + 1 - cmin)/subsample_total*2);
    int copy_height= ((rmax + 1 - rmin)/subsample_total*2);
    copy_width += ((cmax + 1 - cmin)%subsample_total != 0)?1+((cmax + 1 - cmin)%subsample_total)/(copy_cfg.subsample_major+1):0;
    copy_height += ((rmax + 1 - rmin)%subsample_total != 0)?1+((rmax + 1 - rmin)%subsample_total)/(copy_cfg.subsample_major+1):0;

    Mat frame_copy;

    if(color_space == "YUYV" && copy_cfg.input_channels==4)
        frame_copy.create(copy_height, copy_width, CV_8UC2);
    else if(copy_cfg.input_channels==3)
        frame_copy.create(copy_height, copy_width, CV_8UC3);
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Image source %s: unsupported channels %d for subsampled copying.",
            stage_name.c_str(), copy_cfg.input_channels);
        exit(1);
    }

    copy_to_buffer<uint8_t>(frame_copy.ptr<uint8_t>(0),  copy_cfg);

    return frame_copy;
}


void ImageSource::publish_image(long int timestamp)
{
    if(publishing_in_progress)
    {
        RCLCPP_WARN(this->get_logger(), "ImageSource %s publishing backlog. Skipping frame.", stage_name.c_str());
        return;
    }
    else
        publishing_in_progress = true;
    std::chrono::high_resolution_clock::time_point publish_begin = std::chrono::high_resolution_clock::now();
    auto frame = get_image_matrix();
    struct CopyConfig copy_cfg;
    copy_cfg.subsample_major = publish_subsample_major;
    copy_cfg.subsample_minor = publish_subsample_minor;

    Mat frame_copy;
    if(publish_subsample_major!=1 || publish_subsample_minor!=1)
        frame_copy = copy_to_mat(copy_cfg);
    else
        frame_copy = get_image_matrix().clone();

    auto message = msg_ImageSource();

    message.source = stage_name;
    message.image_width = frame_copy.cols;
    message.image_height = frame_copy.rows;
    message.color_space = color_space;
    message.timestamp = timestamp;
    message.frame = frame_count;

    message.image.assign(frame_copy.ptr<uint8_t>(0), frame_copy.ptr<uint8_t>(0)+frame_copy.total()*frame_copy.channels());

    publisher_->publish(message);
    publishing_in_progress = false;
    std::chrono::high_resolution_clock::time_point publish_end = std::chrono::high_resolution_clock::now();
    RCLCPP_DEBUG(this->get_logger(), "ImageSource %s total publish time %.1f ms", stage_name.c_str(),
                std::chrono::duration_cast<std::chrono::microseconds>(publish_end - publish_begin).count()/1000.0);
}
