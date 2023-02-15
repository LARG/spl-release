#pragma once

#include <localization/LocalizationConfig.h>

class TextLogger;
class MemoryCache;
class LocalizationConfig;

#define BASE_DECLARATIONS const MemoryCache& cache, TextLogger*& tlogger, const LocalizationConfig& config
#define BASE_ARGS cache, tlogger, config

class LocalizationBase {
  public:
    LocalizationBase(BASE_DECLARATIONS);
  protected:
    const MemoryCache& cache_;
    TextLogger*& tlogger_;
    const LocalizationConfig& config_;
};
