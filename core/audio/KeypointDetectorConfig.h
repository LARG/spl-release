#pragma once

#include <common/YamlConfig.h>

class KeypointGroup;

class KeypointDetectorConfig : public YamlConfig {
  public:
    KeypointDetectorConfig();
    ~KeypointDetectorConfig();

    float threshold;
    std::vector<KeypointGroup> keypoint_groups;

  private:
    void deserialize(const YAML::Node& node) override;
    void serialize(YAML::Emitter& emitter) const override;
};
