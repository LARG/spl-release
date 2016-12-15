#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <deque>

class KangarooDetector;
class KeypointDetector;
class FFT;
class AudioDetector;

class AudioModule: public Module {
  public:
    AudioModule();
    ~AudioModule();

    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void initFromMemory();
    void processFrame();
    void processCommunications();
    AudioDetector* detector();

  private:
    unsigned long long timestamp_ = 0;
    MemoryCache cache_;
    TextLogger*& tlogger_;
    int whistle_heard_frame_;
    std::deque<float> scores_;
};
