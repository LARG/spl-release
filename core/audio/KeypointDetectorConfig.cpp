#include <audio/KeypointDetectorConfig.h>
#include <audio/KeypointDetector.h>

KeypointDetectorConfig::KeypointDetectorConfig() {
}

KeypointDetectorConfig::~KeypointDetectorConfig() {
}

void KeypointDetectorConfig::deserialize(const YAML::Node& node) {
  YAML_DESERIALIZE(node, threshold);
  YAML_DESERIALIZE(node, keypoint_groups);
}

void KeypointDetectorConfig::serialize(YAML::Emitter& emitter) const {
  YAML_SERIALIZE(emitter, threshold);
  YAML_SERIALIZE(emitter, keypoint_groups);
}
