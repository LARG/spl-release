#include "GyroConfig.h"

GyroConfig::GyroConfig() {
  offsetX = 0.0;
  offsetY = 0.0;
  offsetZ = 0.0;
  calibration_write_time = -1.0;
  slip_amount = 0.000f; // was 0.005f          // roughly 30 degrees per second
  pessimistic_scale = 1.0f;     // never over report to localisation
}

void GyroConfig::deserialize(const YAML::Node& node) {
  YAML_DESERIALIZE(node, offsetX);
  YAML_DESERIALIZE(node, offsetY);
  YAML_DESERIALIZE(node, offsetZ);
  YAML_DESERIALIZE(node, calibration_write_time);
  YAML_DESERIALIZE(node, slip_amount);
  YAML_DESERIALIZE(node, pessimistic_scale);
}

void GyroConfig::serialize(YAML::Emitter& emitter) const {
  YAML_SERIALIZE(emitter, offsetX);
  YAML_SERIALIZE(emitter, offsetY);
  YAML_SERIALIZE(emitter, offsetZ);
  YAML_SERIALIZE(emitter, calibration_write_time);
  YAML_SERIALIZE(emitter, slip_amount);
  YAML_SERIALIZE(emitter, pessimistic_scale);
}
