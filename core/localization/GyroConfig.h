#ifndef GYROCONFIG_H
#define GYROCONFIG_H

#include <common/InterfaceInfo.h>
#include <common/YamlConfig.h>

class GyroConfig : public YamlConfig {

  public:
    GyroConfig();

    double offsetX;
    double offsetY;
    double offsetZ;
    double calibration_write_time;

    // rotational odometry
    double slip_amount;
    double pessimistic_scale;

  private:
    void deserialize(const YAML::Node& node) override;
    void serialize(YAML::Emitter& emitter) const override;
};
#endif
