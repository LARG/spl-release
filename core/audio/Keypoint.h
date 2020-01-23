#pragma once

#include <common/YamlConfig.h>
#include <common/Enum.h>

class Keypoint : public YamlConfig {
  public:
    ENUM_CLASS(Statistic,
      Max,
      Min,
      Mean
    );
    ENUM_CLASS(Type,
      Peak,
      Valley
    );
    float fstart, fend, db_mean, db_stddev, dbz_mean, dbz_stddev;
    float weight;
    Type type;
    Statistic statistic;
    mutable std::string s_type, s_statistic;

    Keypoint() = default;

    Keypoint(float fstart, float fend, 
                float dbMean, float dbStdDev, 
                float dbzMean, float dbzStdDev,
                float weight, Type type,
                Statistic statistic = Statistic::Mean);

    Keypoint(std::vector<float> v);

  private:
    void deserialize(const YAML::Node& node) override;
    void serialize(YAML::Emitter& emitter) const override;
};
