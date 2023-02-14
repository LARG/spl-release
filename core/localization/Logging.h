#include <memory/TextLogger.h>
#define tlog(level, fstring, ...) tlogger_->logFromLocalization(level, fstring, ##__VA_ARGS__)
