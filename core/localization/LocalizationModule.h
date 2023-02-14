#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/ParallelParams.h>
#include <localization/LocalizationConfig.h>
#include <math/Geometry.h>

class LocalizationModule : public Module {
  public:
    LocalizationModule() : tlogger_(textlogger) { }
    virtual void initSpecificModule() = 0;
    void specifyMemoryDependency();
    void specifyMemoryBlocks();

    virtual void reInit() = 0;
    virtual void initFromMemory() = 0;
    virtual void initFromWorld() { }
    virtual void processFrame() = 0;

    void filterCloseBallPosition(float offset_x, float offset_y, float scale_y);
    Point2D filtered_close_ball_;

    virtual void loadParallelParams(ParallelParams params) { }

    int method;
    
    virtual void moveBall(const Point2D& position) = 0;
    virtual void movePlayer(const Point2D& position, float orientation) = 0;

  protected:
    virtual void initMemory() { }
    void initConfig();

  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationConfig config_;
};

#endif
