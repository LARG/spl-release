#include <ObjectDetector.hpp>

ObjectDetector::ObjectDetector(string name_, Json::Value config) : VisionStage(name_, config)
{
    configure(config);
    // Initialize publisher for detected objects
    objects_publisher = this->create_publisher<msg_DetectedObjects>(TOPIC_VISION_DETECTED_OBJECTS, 10);
}


void ObjectDetector::configure(Json::Value config)
{
    ReadJSONAttribute(config,image_source_name,String);
    object_map = {IO_BALL, IO_CROSS, IO_ROBOT};
    if(config.isMember("object_map"))
    {
      object_map.clear();
      for(auto ob_id: config["object_map"])
        object_map.push_back(object_names_map[ob_id.asString()]);
    }
}


void ObjectDetector::execute(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
    if(stages.find(image_source_name) == stages.end())
    {
        RCLCPP_ERROR(this->get_logger(), "ObjectDetector '%s': ImageSource not found: '%s'", stage_name.c_str(), image_source_name.c_str());
        exit(1);
    }

    // Perform detection
    std::chrono::high_resolution_clock::time_point detect_start_time = std::chrono::high_resolution_clock::now();
    vector<msg_ImageObject> detected_objects = detect(stages);
    std::chrono::high_resolution_clock::time_point detect_end_time = std::chrono::high_resolution_clock::now();

    frame_count++;

    // TODO: full profiling support
    if(frame_count%30==0)
    RCLCPP_DEBUG(this->get_logger(), "Count %d Detection time: %.1f ms", (int)frame_count,
                std::chrono::duration_cast<std::chrono::microseconds>(detect_end_time - detect_start_time).count()/1000.0);

    // Add metadata to objects
    for(auto object_itr = detected_objects.begin(); object_itr < detected_objects.end(); ++object_itr)
    {
        object_itr->source = stage_name;
        object_itr->frame = frame_count;
    }

    // Publish detected objects
    auto detection_msg = msg_DetectedObjects();
    detection_msg.objects = detected_objects;
    objects_publisher->publish(detection_msg);
}
