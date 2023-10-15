#include <LineDetector.hpp>

LineDetector::LineDetector(string& name_, Json::Value& config) : VisionStage(name_, config)
{
    configure(config);
    // Initialize publisher for detected lines
    lines_publisher = this->create_publisher<msg_DetectedLines>(TOPIC_VISION_DETECTED_LINES, 10);
}


void LineDetector::configure(Json::Value config)
{
    ReadJSONAttribute(config,image_source_name,String);
}


void LineDetector::execute(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
    if(stages.find(image_source_name) == stages.end())
    {
        RCLCPP_ERROR(this->get_logger(), "ObjectDetector '%s': ImageSource not found: '%s'", stage_name.c_str(), image_source_name.c_str());
        exit(1);
    }

    // Perform detection
    std::chrono::high_resolution_clock::time_point detect_start_time = std::chrono::high_resolution_clock::now();
    vector<msg_ImageLine> detected_lines = detect(stages);
    std::chrono::high_resolution_clock::time_point detect_end_time = std::chrono::high_resolution_clock::now();

    frame_count++;

    // TODO: full profiling support
    if(frame_count%30==0)
    RCLCPP_DEBUG(this->get_logger(), "Line detector %s: Count %d Detection time: %.1f ms", stage_name.c_str(), (int)frame_count,
                std::chrono::duration_cast<std::chrono::microseconds>(detect_end_time - detect_start_time).count()/1000.0);

    // Add metadata to objects
    for(auto line_itr = detected_lines.begin(); line_itr < detected_lines.end(); ++line_itr)
    {
        line_itr->source = stage_name;
        line_itr->frame = frame_count;
    }

    // Publish detected objects
    auto detection_msg = msg_DetectedLines();
    detection_msg.lines = detected_lines;
    lines_publisher->publish(detection_msg);
}
