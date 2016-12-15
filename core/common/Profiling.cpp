#include <common/Profiling.h>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>
#define FIRST_ID 1
static int __id = FIRST_ID;
static int __lastid = __id;
static std::unordered_map<int,Timer::TimePoint> __times;
static std::unordered_map<int,TimeList> __timelists;
constexpr int Timer::DefaultInterval;
constexpr int Timer::DefaultLevel;
std::unordered_map<std::string,std::unique_ptr<Timer>> Timer::timers_;

using T = std::remove_const_t<decltype(Timer::EnabledModules)>;
using M = Timer::Module;
#define INDEX(m) static_cast<int>(m)
#define ECHO(z,n,data) n,
#define LEVEL_RANGE(from,to) BOOST_PP_REPEAT_FROM_TO(from,to,ECHO,"")

// TIMER FILTERING: 
const std::unordered_set<int> Timer::EnabledLevels = {
  // Enter timer levels here that you want 
  // to explicitly allow
  // 
  // Ex:
  //  LEVEL_RANGE(20,30) // Allow all levels from 20 to 30
  //  5, 18, 3 // Allow levels 5, 18, and 3
};
const T Timer::EnabledModules = [] {
  T a;  
  // Enable modules by setting their indexes to 1
  // Ex:
  a[static_cast<int>(M::Vision)] = 1; // Enable the Vision module
  a[static_cast<int>(M::Audio)] = 1; // Enable the Audio module
  return a;
}();

#ifdef OPTIMIZE
constexpr bool Timer::EnableProfiling;
#else
const bool Timer::EnableProfiling = false;
#endif

std::unordered_map<std::string,int> Timer::TimerLevels;
std::unordered_map<std::string,Timer::State> Timer::TimerState;

int tic(int id) {
  auto t = std::chrono::time_point_cast<Timer::Duration>(Timer::Clock::now());
  if(id >= FIRST_ID) {
    __times[id] = t;
    return id;
  }
  else {
    __times[__id] = t;
    __lastid = __id;
    return __id++;
  }
}

Timer::Duration __toc(int id) {
  if(id < 0) id = __lastid;
  auto tstart = __times[id];
  auto tend = Timer::Clock::now();
  auto elapsed = std::chrono::duration_cast<Timer::Duration>(tend - tstart);
  return elapsed;
}

double toc(int id) {
  auto elapsed = __toc(id);
  auto elapsed_s = std::chrono::duration_cast<Timer::Seconds>(elapsed);
  return elapsed_s.count();
}

double toc_s(int id) {
  return toc(id);
}

double toc_ms(int id) {
  auto elapsed = toc(id);
  auto elapsed_s = std::chrono::duration_cast<Timer::Milliseconds>(__toc(id));
  return elapsed_s.count();
}

void printtime(int id) {
  if(!Timer::EnableProfiling) return;
  auto seconds = toc_s(id);
  setlocale(LC_NUMERIC, "");
  printf("time: %'2.10fs\n", seconds);
}

void printtime(const char* name, int id) {
  if(!Timer::EnableProfiling) return;
  printf("%s ", name);
  printtime(id);
}

Timer::Timer() : id_(0), pauseId_(0), elapsed_(0), paused_(0), interval_(1), iterations_(0) { }

Timer::Timer(std::string message, int interval) : Timer() {
  interval_ = interval;
  message_ = message;
}

void Timer::start() {
  if(!id_) id_ = tic();
  else tic(id_);
}

void Timer::stop() {
  elapsed_ += __toc(id_);
  elapsed_ -= paused_;
  paused_ = Timer::Duration(0);
  int i = (iterations_ + 1) % (interval_ + 1);
  if(i == 0) restart();
  else iterations_ = i;
}

void Timer::pause() {
  if(pauseId_) tic(pauseId_);
  else pauseId_ = tic();
}

void Timer::unpause() {
  paused_ += __toc(pauseId_);
}

bool Timer::ready() {
  return iterations_ == interval_;
}

void Timer::restart() {
  iterations_ = 0;
  elapsed_ = Timer::Duration(0);
  tic(id_);
}

double Timer::avgrate() {
  double elapsed = static_cast<double>(elapsed_.count()) / Timer::Duration::period::den;
  return interval_ / elapsed;
}

int32_t Timer::avgtime_us() {
  return (elapsed_ / interval_).count();
}

int32_t Timer::elapsed_us() {
  return __toc(id_).count();
}

double Timer::avgtime_ms() {
  return std::chrono::duration_cast<Timer::Milliseconds>(elapsed_ / interval_).count();
}

double Timer::elapsed_ms() {
  return std::chrono::duration_cast<Timer::Milliseconds>(__toc(id_)).count();
}

double Timer::avgtime_s() {
  return std::chrono::duration_cast<Timer::Seconds>(elapsed_ / interval_).count();
}

double Timer::elapsed_s() {
  return std::chrono::duration_cast<Timer::Seconds>(__toc(id_)).count();
}

void Timer::setMessage(std::string message) {
  message_ = message;
}

void Timer::setInterval(int interval) {
  interval_ = interval;
}

void Timer::printAtInterval() {
  if(!Timer::EnableProfiling) return;
  if(!ready()) return;
  setlocale(LC_NUMERIC, "");
  if(message_.size() > 0)
    printf("%s: %'2.3fms\n", message_.c_str(), avgtime_ms());
  else
    printf("%'2.3fms\n", avgtime_ms());
  restart();
}
