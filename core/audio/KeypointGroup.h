#pragma once

#include <common/YamlConfig.h>
#include <memory/AudioProcessingBlock.h>
#include <audio/Keypoint.h>

class KeypointGroup : public YamlConfig {
  public:
    KeypointGroup(std::string name, std::vector<Keypoint> keypoints);
    KeypointGroup();
    ~KeypointGroup();

    std::string name;
    std::vector<Keypoint> keypoints;

  private:
    void deserialize(const YAML::Node& node) override;
    void serialize(YAML::Emitter& emitter) const override;
};
