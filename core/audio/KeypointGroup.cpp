#include <audio/KeypointGroup.h>

KeypointGroup::KeypointGroup() {
}

KeypointGroup::~KeypointGroup() {
}

void KeypointGroup::deserialize(const YAML::Node& node) {
  YAML_DESERIALIZE(node, name);
  YAML_DESERIALIZE(node, keypoints);
}

void KeypointGroup::serialize(YAML::Emitter& emitter) const {
  YAML_SERIALIZE(emitter, name);
  YAML_SERIALIZE(emitter, keypoints);
}
