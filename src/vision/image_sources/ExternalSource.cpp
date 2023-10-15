#include <ExternalSource.hpp>

using std::placeholders::_1;

std::mutex ext_src_lock;

ExternalSource::ExternalSource(string& name_, Json::Value& config) : ImageSource(name_, config)
{
    configure(config);
    publisher_ = this->create_publisher<msg_String>("villa/"+service_name+"/request", 10);
    subscription_ = this->create_subscription<msg_ImageSource>(
     "villa/"+service_name+"/response" , 10, std::bind(&ExternalSource::response_callback, this, _1));
    // ext_src_lock.lock();
}


void ExternalSource::configure(Json::Value config)
{
    ImageSource::configure(config);

    ReadJSONAttribute(cfg, service_name, String);

    client = this->create_client<common::srv::GetImage>(service_name);
}


void ExternalSource::acquire_image(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
    auto request = std::make_shared<common::srv::GetImage::Request>();
    // Don't need to set anything in the request

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    // if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) !=
    //     rclcpp::FutureReturnCode::SUCCESS)
    if(result.wait_for(2s) != std::future_status::ready)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", service_name.c_str());
        exit(1);
    }
    auto image_msg = result.get()->image;

    Mat(image_height, image_width,
        (color_space=="YUYV"?CV_8UC2:CV_8UC3),
        image_msg.image.data()
       ).copyTo(raw_image);

    // auto req_msg = msg_String();
    // publisher_->publish(req_msg);
    // ext_src_lock.lock();
}

void ExternalSource::response_callback(msg_ImageSource img_msg)
{
    Mat(image_height, image_width,
        (color_space=="YUYV"?CV_8UC2:CV_8UC3),
        img_msg.image.data()
       ).copyTo(raw_image);
    ext_src_lock.unlock();
}


Mat& ExternalSource::get_image_matrix()
{
    return raw_image;
}
