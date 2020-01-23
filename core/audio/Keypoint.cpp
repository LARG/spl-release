#include <audio/Keypoint.h>
Keypoint::Keypoint(float fstart, float fend, 
                         float dbMean, float dbStdDev, 
                         float dbzMean, float dbzStdDev,
                         float weight, Type type,
                         Statistic statistic) :
  fstart(fstart), fend(fend), 
  db_mean(dbMean), db_stddev(dbStdDev), 
  dbz_mean(dbzMean), dbz_stddev(dbzStdDev), 
  weight(weight), type(type),
  statistic(statistic) {
}

Keypoint::Keypoint(std::vector<float> v) {
  int i = 0;
  fstart = v[i++]; fend = v[i++];
  db_mean = v[i++]; db_stddev = v[i++];
  dbz_mean = v[i++]; dbz_stddev = v[i++];
  weight = v[i++];
  type = static_cast<Type>(v[i++]);
  statistic = static_cast<Statistic>(v[i++]);
}

void Keypoint::deserialize(const YAML::Node& node) {
  YAML_DESERIALIZE(node, fstart);
  YAML_DESERIALIZE(node, fend);
  YAML_DESERIALIZE(node, db_mean);
  YAML_DESERIALIZE(node, db_stddev);
  YAML_DESERIALIZE(node, dbz_mean);
  YAML_DESERIALIZE(node, dbz_stddev);
  YAML_DESERIALIZE(node, weight);
  YAML_DESERIALIZE(node, s_type);
  YAML_DESERIALIZE(node, s_statistic);
  type = TypeMethods::fromName(s_type);
  statistic = StatisticMethods::fromName(s_statistic);
}

void Keypoint::serialize(YAML::Emitter& emitter) const {
  s_type = TypeMethods::getName(type);
  s_statistic = StatisticMethods::getName(statistic);
  YAML_SERIALIZE(emitter, fstart);
  YAML_SERIALIZE(emitter, fend);
  YAML_SERIALIZE(emitter, db_mean);
  YAML_SERIALIZE(emitter, db_stddev);
  YAML_SERIALIZE(emitter, dbz_mean);
  YAML_SERIALIZE(emitter, dbz_stddev);
  YAML_SERIALIZE(emitter, weight);
  YAML_SERIALIZE(emitter, s_type);
  YAML_SERIALIZE(emitter, s_statistic);
}
