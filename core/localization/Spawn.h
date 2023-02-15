#ifndef SPAWN_H
#define SPAWN_H

#include <map>
#include <common/RobotPositions.h>
#include <memory/MemoryCache.h>
#include <memory/TextLogger.h>
#include <memory/LocalizationBlock.h>
#include <localization/ParallelUKF.h>
#include <localization/Disambiguator.h>
#include <localization/PoseEstimate.h>
#include <common/Enum.h>

struct SpawnedModel {
  ParallelUKF* model;
  ObjectSet objects;
  SpawnedModel(ParallelUKF* model, ObjectSet objects) : model(model), objects(objects) { };
};

class Spawn : public LocalizationBase {
  public:
    Spawn(BASE_DECLARATIONS, std::vector<ParallelUKF*>& models, ParallelParams& params);
    std::vector<SpawnedModel> attempt(ParallelUKF* model, std::vector<Spawns::SpawnType> types, const ParallelObservation& obs, const vector<ObjectSet>& objectSets);
    void initFromMemory();
    void reset();

  protected:
    PoseEstimate computePoseEstimate(const std::vector<SeenObject>& objects);
    std::vector<SpawnedModel> attempt(ParallelUKF* model, Spawns::SpawnType type, const ParallelObservation& obs, const vector<ObjectSet>& objectSets);
    bool ready(Spawns::SpawnType type);
    void complete(Spawns::SpawnType type);
    vector<ParallelUKF*> flip(ParallelUKF* model);

  private:
    ParallelParams& params_;
    std::vector<ParallelUKF*>& models_;
    std::map<Spawns::SpawnType, int> frames_;
    std::map<Spawns::SpawnType, int> minframes_;
    bool delete_;
};

#endif
