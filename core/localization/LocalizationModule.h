#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <Module.h>
#include <memory/MemoryCache.h>
#include <math/Geometry.h>

class LocalizationModule : public Module {
  public:
    LocalizationModule() : tlogger_(textlogger) { }
    virtual void initSpecificModule() { }
    void specifyMemoryDependency();
    void specifyMemoryBlocks();

    virtual void reInit() { }
    virtual void initFromMemory() { }
    virtual void initFromWorld() { }
    virtual void processFrame() { }

    void filterCloseBallPosition(float offset_x, float offset_y, float scale_y);
    Point2D filtered_close_ball_;

    int method;
    
  protected:
    virtual void initMemory() { }
    void initConfig();

  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
};

#endif
