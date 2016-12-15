#pragma once

#ifndef SWIG

#define FORMATTABLE_METHOD(MethodName) \
    template<typename... Ts> \
    inline static void MethodName(Module module, int level, const char* format, Ts&&... ts) { \
      if(Ignore(level, module)) return; \
      auto name = tfm::format(format, std::forward<Ts>(ts)...); \
      MethodName(std::move(name), level, module); \
    } \
    template<typename... Ts> \
    inline static void MethodName(Module module, int level, const std::string& format, Ts&&... ts) { \
      if(Ignore(level, module)) return; \
      auto name = tfm::format(format, std::forward<Ts>(ts)...); \
      MethodName(std::move(name), level, module); \
    } \
    template<typename... Ts> \
    inline static void MethodName(Module module, const char* format, Ts&&... ts) { \
      constexpr int level = 0; \
      if(Ignore(level, module)) return; \
      auto name = tfm::format(format, std::forward<Ts>(ts)...); \
      MethodName(std::move(name), level, module); \
    } \
    template<typename... Ts> \
    inline static void MethodName(Module module, const std::string& format, Ts&&... ts) { \
      constexpr int level = 0; \
      if(Ignore(level, module)) return; \
      auto name = tfm::format(format, std::forward<Ts>(ts)...); \
      MethodName(std::move(name), level, module); \
    } \
    template<typename... Ts> \
    inline static void MethodName(int level, const char* format, Ts&&... ts) { \
      constexpr Module module = Module::Unspecified; \
      if(Ignore(level, module)) return; \
      auto name = tfm::format(format, std::forward<Ts>(ts)...); \
      MethodName(std::move(name), level, module); \
    } \
    template<typename... Ts> \
    inline static void MethodName(int level, const std::string& format, Ts&&... ts) { \
      constexpr Module module = Module::Unspecified; \
      if(Ignore(level, module)) return; \
      auto name = tfm::format(format, std::forward<Ts>(ts)...); \
      MethodName(std::move(name), level, module); \
    } \
    template<typename... Ts> \
    inline static void MethodName(const char* format, Ts&&... ts) { \
      constexpr int level = 0; \
      constexpr Module module = Module::Unspecified; \
      if(Ignore(level, module)) return; \
      auto name = tfm::format(format, std::forward<Ts>(ts)...); \
      MethodName(std::move(name), level, module); \
    } \
    template<typename... Ts> \
    inline static void MethodName(const std::string& format, Ts&&... ts) { \
      constexpr int level = 0; \
      constexpr Module module = Module::Unspecified; \
      if(Ignore(level, module)) return; \
      auto name = tfm::format(format, std::forward<Ts>(ts)...); \
      MethodName(std::move(name), level, module); \
    }

#define MODULE_METHOD(ModuleName, MethodName) \
  template<typename... Ts> \
  inline static void MethodName(Ts&&... ts) { \
    Timer::MethodName(Timer::Module::ModuleName, std::forward<Ts>(ts)...); \
  }

#define MODULE_TIMER(ModuleName) \
namespace ModuleName##Timer { \
  MODULE_METHOD(ModuleName, Start); \
  MODULE_METHOD(ModuleName, Stop); \
  MODULE_METHOD(ModuleName, Pause); \
  MODULE_METHOD(ModuleName, Unpause); \
  MODULE_METHOD(ModuleName, StartPause); \
  MODULE_METHOD(ModuleName, Wrap); \
}

#endif
