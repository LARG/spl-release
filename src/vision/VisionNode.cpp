#include <VisionNode.hpp>

using std::placeholders::_1;


template<typename stage_T> shared_ptr<VisionStage> init_vision_stage(string& name_, Json::Value& cfg_)
{
    VisionStage * stage_ptr = (VisionStage*) new stage_T(name_, cfg_);
    shared_ptr<VisionStage> stage_shrptr(stage_ptr);

    return stage_shrptr;
}


VisionNode :: VisionNode() : Node("vision_node")
{
    RCLCPP_DEBUG(this->get_logger(), "Initializing Vision Node.");

    this->declare_parameter("config_path", VISION_NODE_CONFIG_PATH);

    string config_file_path = this->get_parameter("config_path").as_string();
    
    // Load config
    RCLCPP_DEBUG(this->get_logger(), "Loading config from path: %s", config_file_path.c_str());

    std::ifstream ifs;
    ifs.open(config_file_path.c_str());
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, ifs, &default_config, &errs)) {
        std::cout << errs << std::endl;
        exit(1);
    }

    RCLCPP_DEBUG(this->get_logger(), "Successfully loaded config.");
    
    // Initialize vision stages. The configs are
    // expected to be present as key-value pairs.
    for (auto it = default_config.begin(); it != default_config.end(); it++)
    {
        string stage_name = it.key().asString();
        RCLCPP_DEBUG(this->get_logger(), "Initializing vision stage: %s", stage_name.c_str());

        Json::Value cfg = *it;

        // Find out what the type of the vision stage is and initialize
        // accordingly.
        
        shared_ptr<VisionStage> vision_stage;
        if(cfg["stage_type"] == "CameraCapture")
            vision_stage = init_vision_stage<CameraCapture>(stage_name, cfg);
        else if(cfg["stage_type"] == "StaticImage")
            vision_stage = init_vision_stage<StaticImage>(stage_name, cfg);
        else if(cfg["stage_type"] == "ExternalSource")
            vision_stage = init_vision_stage<ExternalSource>(stage_name, cfg);
        else if(cfg["stage_type"] == "TFLiteDetector")
            vision_stage = init_vision_stage<TFLiteDetector>(stage_name, cfg);
        else if(cfg["stage_type"] == "HoughLineDetector")
            vision_stage = init_vision_stage<HoughLineDetector>(stage_name, cfg);
        else if(cfg["stage_type"] == "ColorSegmenter")
            vision_stage = init_vision_stage<ColorSegmenter>(stage_name, cfg);
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Vision initialization failed for stage %s: Unsupported stage type: %s", stage_name.c_str(), cfg["stage_type"].asString().c_str());
            exit(1);
        }

        // Add to the vision node's collection of stages

        vision_stages[stage_name] = vision_stage; //TODO: Make shared pointer again
    }

    // Initialize topic subscriptions
    invoke_subscription = this->create_subscription<msg_String>(
      TOPIC_VISION_INVOKE, 10, std::bind(&VisionNode::invoke_callback, this, _1));
    
    reconfigure_subscription = this->create_subscription<msg_String>(
      TOPIC_VISION_RECONFIGURE, 10, std::bind(&VisionNode::reconfigure_callback, this, _1));
}


void VisionNode :: invoke_callback(msg_InvokeVision invoke_message)
{
    // Invoke using ros2 utils:
    //       ros2 topic pub --rate 30 --print 100 "villa/vision/invoke" std_msgs/String '{data : '\''[{ "name" : "nao_camera_top"} , {"name" : "ball_detector1"}]'\''}'

    Json::Value invoke_configs;

    // Parse the invoke message as JSON.
    Json::CharReaderBuilder builder;
    JSONCPP_STRING err;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(invoke_message.data.c_str(), invoke_message.data.c_str() + invoke_message.data.length(), &invoke_configs,
                       &err)) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing invoke config: ");
        cerr << err << endl;
        exit(1);
    }

    // Run stages
    for (auto stage_invoke_config : invoke_configs)
    {
        string invoke_name = stage_invoke_config["name"].asString();
        if (vision_stages.find(invoke_name) == vision_stages.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Stage not found: '%s'", invoke_name.c_str());
            exit(1);
        }

        RCLCPP_DEBUG(this->get_logger(), "Running vision stage: %s", invoke_name.c_str());
        auto stage = vision_stages[invoke_name];

        stage -> invoke_cfg = stage_invoke_config;

        // Invoke and time
        std::chrono::high_resolution_clock::time_point stage_start_time = std::chrono::high_resolution_clock::now();
        stage -> execute(vision_stages); //TODO : also pass in invoke config
        std::chrono::high_resolution_clock::time_point stage_end_time = std::chrono::high_resolution_clock::now();
        
        // TODO: global frame counter, profiling module
        RCLCPP_DEBUG(this->get_logger(), "Vision stage %s: Execution time: %.1f ms", invoke_name.c_str(),
                std::chrono::duration_cast<std::chrono::microseconds>(stage_end_time - stage_start_time).count()/1000.0);
    }
}


void VisionNode::reconfigure_callback(msg_String reconfigure_message)
{
    // Reconfigure using ros2 utils:
    //
    // ros2 topic pub "villa/vision/reconfigure" std_msgs/String '{data : '\''[{ "name" : "nao_camera_top", "publish_subsample_major": 10, "publish_subsample_minor": 10}]'\''}' --once

    Json::Value config_updates;
    // Parse the invoke message as JSON.
    Json::CharReaderBuilder builder;
    JSONCPP_STRING err;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(reconfigure_message.data.c_str(), reconfigure_message.data.c_str() + reconfigure_message.data.length(), &config_updates,
                       &err)) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing config update: ");
        cerr << err << endl;
        exit(1);
    }

    for (auto stage_config : config_updates)
    {
        string conf_name = stage_config["name"].asString();
        if (vision_stages.find(conf_name) == vision_stages.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Stage not found: '%s'", conf_name.c_str());
            exit(1);
        }
        RCLCPP_DEBUG(this->get_logger(), "Reconfiguring vision stage: %s", conf_name.c_str());
        auto stage = vision_stages[conf_name];
        stage->reconfigure(stage_config);
    }
}

void VisionNode::register_stages(rclcpp::executors::MultiThreadedExecutor& exc)
{
    for(auto it=vision_stages.begin(); it != vision_stages.end(); it++)
    {
        exc.add_node(it->second);
    }
}
