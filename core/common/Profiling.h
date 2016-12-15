#pragma once

#include <unordered_map>
#include <unordered_set>
#include <list>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <clocale>
#include <memory>
#include <common/tinyformat.h>
#include <common/ProfilingMacros.h>
#include <common/Enum.h>

class Timer {
  public:
    ENUM_CLASS(Module,
      Unspecified,
      Vision,
      Loc,
      Audio
    );
    ENUM_CLASS(State,
      Started,
      Stopped
    );
#ifndef SWIG
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<int, std::ratio<1, 1'000'000>>;
    using TimePoint = std::chrono::time_point<Clock, Duration>;
    using Seconds = std::chrono::duration<double, std::ratio<1>>;
    using Milliseconds = std::chrono::duration<double, std::milli>;
#endif
    static constexpr int DefaultInterval = 90;
    static constexpr int DefaultLevel = 0;
    Timer();
    Timer(std::string message, int interval=DefaultInterval);
    void start();
    void stop();
    inline void stopPrint() { stop(); printAtInterval(); }
    void pause();
    void unpause();
    void restart();
    void setMessage(std::string message);
    void setInterval(int interval);
    void printAtInterval();
    bool ready();
    double avgrate();
    int32_t avgtime_us();
    int32_t elapsed_us();
    double avgtime_ms();
    double elapsed_ms();
    double avgtime_s();
    double elapsed_s();

#ifdef OPTIMIZE
    static constexpr bool EnableProfiling = false;
#else
    static const bool EnableProfiling;
#endif
    static const std::unordered_set<int> EnabledLevels;
    // This should be a vector<bool> except that vector<bool> is broken
    // in standard C++. See https://isocpp.org/blog/2012/11/on-vectorbool
    static const std::array<int,static_cast<long unsigned int>(Module::NUM_Modules)> EnabledModules;
    static std::unordered_map<std::string,int> TimerLevels;
    static std::unordered_map<std::string,State> TimerState;

#ifndef SWIG
    FORMATTABLE_METHOD(Start);
    FORMATTABLE_METHOD(Pause);
    FORMATTABLE_METHOD(StartPause);
    FORMATTABLE_METHOD(Unpause);
    FORMATTABLE_METHOD(Stop);
    FORMATTABLE_METHOD(Wrap);

  private:
    inline static Timer* Start(std::string&& name, int level, Module module) {
      Timer* t = nullptr;
      if(Ignore(name, level, module)) return t;
      auto it = timers_.find(name);
      if(it == timers_.end()) {
        auto timer = std::make_unique<Timer>(name, DefaultInterval);
        t = timer.get();
        timers_[name] = std::move(timer);
      } else t = it->second.get();
      t->start();
      return t;
    }

    inline static void StartPause(std::string&& name, int level, Module module) {
      if(Ignore(name, level, module)) return;
      Timer* t = Start(std::move(name), level, module);
      t->pause();
    }

    inline static void Pause(std::string&& name, int level, Module module) {
      if(Ignore(name, level, module)) return;
      auto it = timers_.find(name);
      if(it == timers_.end()) return;
      it->second->pause();
    }
    
    inline static void Unpause(std::string&& name, int level, Module module) {
      if(Ignore(name, level, module)) return;
      auto it = timers_.find(name);
      if(it == timers_.end()) return;
      it->second->unpause();
    }

    inline static void Stop(std::string&& name, int level, Module module) {
      if(Ignore(name, level, module)) return;
      auto it = timers_.find(name);
      if(it == timers_.end()) return;
      it->second->stopPrint();
    }

    inline static void Wrap(std::string&& name, int level, Module module) {
      if(Ignore(name, level, module)) return;
      auto it = TimerState.find(name);
      if(it == TimerState.end()) {
        Start(std::move(name), level, module);
        TimerState[name] = State::Started;
      }
      else if(it->second == State::Started) {
        Stop(std::move(name), level, module);
        TimerState[name] = State::Stopped;
      } else if (it->second == State::Stopped) {
        Start(std::move(name), level, module);
        TimerState[name] = State::Started;
      }
    }

    inline static bool Ignore(int level, Module module) {
      if(!EnableProfiling) return true;
      static const std::string e = "";
      return Ignore(e, level, module);
    }

    inline static bool Ignore(const std::string& name, int level, Module module) {
      if(!Timer::EnableProfiling) return true;
      if(Timer::EnabledModules[static_cast<int>(module)] == 0)
        return true;
      if(!name.empty()) {
        if(level == DefaultLevel) {
          auto it = TimerLevels.find(name);
          if(it != TimerLevels.end()) {
            level = it->second;
          }
        } else TimerLevels.emplace(name, level);
      }
      if(!Timer::EnabledLevels.empty() && Timer::EnabledLevels.find(level) == Timer::EnabledLevels.end()) 
        return true;
      return false;
    }
#endif

  private:

    int id_, pauseId_;
    Duration elapsed_, paused_;
    int interval_, iterations_;
    std::string message_;
    static std::unordered_map<std::string,std::unique_ptr<Timer>> timers_;
};

#ifndef SWIG
MODULE_TIMER(Audio);
MODULE_TIMER(Loc);
MODULE_TIMER(Vision);
#endif

struct TimeList {
  int maxSize;
  std::list<Timer::Duration> times;
};

int tic(int id = -1);
Timer::Duration __toc(int id = -1);
double toc(int id = -1);
double toc_s(int id = -1);
double toc_ms(int id = -1);
void printtime(int id = -1);
void printtime(const char* message, int id = -1);
