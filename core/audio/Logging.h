#include <memory/TextLogger.h>
#define tlog(level, fstring, ...) tlogger_->logFromAudio(level, fstring, ##__VA_ARGS__)
