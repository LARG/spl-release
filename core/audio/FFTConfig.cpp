#include <audio/FFTConfig.h>

FFTConfig::FFTConfig() {
  channel_offset = 0;
  window_size = 128;
  window_step = 64;
}

void FFTConfig::deserialize(const YAML::Node& node) {
  YAML_DESERIALIZE(node, channel_offset);
  YAML_DESERIALIZE(node, window_size);
  YAML_DESERIALIZE(node, window_step);
  YAML_DESERIALIZE(node, maximum_samples);
}

void FFTConfig::serialize(YAML::Emitter& emitter) const {
  YAML_SERIALIZE(emitter, channel_offset);
  YAML_SERIALIZE(emitter, window_size);
  YAML_SERIALIZE(emitter, window_step);
  YAML_SERIALIZE(emitter, maximum_samples);
}
