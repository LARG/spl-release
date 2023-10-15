#include <CameraCapture.hpp>



CameraCapture::CameraCapture(string& name_, Json::Value& config) : ImageSource(name_, config)
{
    configure(config);
}


void CameraCapture::configure(Json::Value config)
{
    ImageSource::configure(config);

    // Get necessary details from the config object
    string device_fname = config["device_filename"].asString();

    // Release any opened camera devices (can happen if reconfiguring)
    if (cv_capture.isOpened())
        cv_capture.release();

    // Open the camera device
    cv_capture.open(device_fname, cv::CAP_ANY);
    if (!cv_capture.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "CameraCapture %s: Failed to open device %s.",
            stage_name.c_str(), device_fname.c_str());
        exit(1);
    }

    // Set frame width and height
    cv_capture.set(CAP_PROP_FRAME_WIDTH, image_width);
    cv_capture.set(CAP_PROP_FRAME_HEIGHT, image_height);

    // Do this so that it doesn't spend time converting the YUYV image
    if(color_space == "YUYV")
        cv_capture.set(CAP_PROP_CONVERT_RGB, false);
}


void CameraCapture::acquire_image(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
    // wait for a new frame from camera and store it
    Mat camera_image;
    cv_capture.read(camera_image);

    // check if we succeeded
    if (camera_image.empty()) {
        cerr << "ERROR! blank frame grabbed\n";
    }

    if(flip_horizontal && flip_vertical)
        flip(camera_image, raw_image, -1);
    else
        raw_image = camera_image;
}

Mat& CameraCapture::get_image_matrix()
{
    return raw_image;
}