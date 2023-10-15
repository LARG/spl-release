#include <VisionStage.hpp>


void updateJsonValue(Json::Value& originalValue, const Json::Value& newValue) {
    if (newValue.isObject()) {
        // Iterate over the keys in the new value
        for (Json::Value::const_iterator it = newValue.begin(); it != newValue.end(); ++it) {
            const std::string& key = it.key().asString();
            const Json::Value& newValueForKey = *it;
            
            // If the key exists in the original value, update its value
            if (originalValue.isMember(key)) {
                Json::Value& originalValueForKey = originalValue[key];
                updateJsonValue(originalValueForKey, newValueForKey);
            } else {
                // If the key does not exist in the original value, add it
                originalValue[key] = newValueForKey;
            }
        }
    } else if (newValue.isArray()) {
        // Handle arrays recursively
        for (Json::ArrayIndex i = 0; i < newValue.size(); ++i) {
            if (i < originalValue.size()) {
                updateJsonValue(originalValue[i], newValue[i]);
            } else {
                originalValue.append(newValue[i]);
            }
        }
    } else {
        // Update non-object and non-array values
        originalValue = newValue;
    }
}


VisionStage::VisionStage(string name_, Json::Value init_cfg) : Node(name_.c_str())
{
    stage_name = name_;
    // Store JSON config to use for reconfiguration
    cfg = init_cfg;
}


void VisionStage::reconfigure(Json::Value config)
{
    updateJsonValue(cfg, config);
    configure(cfg);
}