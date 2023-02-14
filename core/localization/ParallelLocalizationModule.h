#ifndef PARALLEL_LOCALIZATION_MODULE_H
#define PARALLEL_LOCALIZATION_MODULE_H

#include <localization/LocalizationModule.h>
#include <algorithm>
#include <common/WorldObject.h>

class Disambiguator;
class BallUKF;
class BallUKFParams;
class Spawn;
class ParallelUKF;
class ParallelObservation;

class ParallelLocalizationModule : public LocalizationModule {
  public:
    ParallelLocalizationModule();
    void initSpecificModule();
    void initFromMemory();
    void initFromWorld();
    void reInit();
    void processFrame();

    void loadParallelParams(ParallelParams params);

    void moveBall(const Point2D& position) override;
    void movePlayer(const Point2D& position, float orientation) override;
  protected:
    void initModels(ParallelUKF* model);
    void initMemory();
  private:
    void processObservations(ParallelUKF* model, ParallelObservation& obs);
    void fillBaseObservations(ParallelUKF* model, ParallelObservation& obs);
    void processCommunications(ParallelUKF* model, ParallelObservation& obs);
    void attemptSplits(ParallelUKF* model, const ParallelObservation& obs, vector<ObjectSet>& objectSets);
    void fillSeenObjects(ParallelUKF* model, ParallelObservation& obs, std::vector<WorldObject>& dobjects);
    void updateCloseBall(const ParallelObservation& obs);
    void processPriorKnowledge(ParallelUKF* model);
    void clipToField(const Point2D& selfLoc, float selfOrientation, WorldObject& object);
    void updateMemory();
    void normalizeModels();
    void selectBestModel();
    void checkErrors();
    std::vector<ParallelUKF*> models_;
    std::vector<WorldObjectType> loctypes_;
    std::list<ParallelUKF*> history_;
    ParallelUKF* best_;
    ParallelParams params_;
    std::unique_ptr<Spawn> spawn_;
    std::unique_ptr<BallUKF> ball_ukf_;
    std::unique_ptr<BallUKFParams> ball_params_;
    std::unique_ptr<Disambiguator> disambiguator_;
    int state_elapsed_frames_;
    float total_turn = 0.0;
};

#endif
