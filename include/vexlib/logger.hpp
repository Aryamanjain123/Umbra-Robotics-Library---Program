#pragma once
#include <cstdarg>
#include <cstdio>
#include <string>
#include "vexlib/base.hpp"

namespace vexlib {

class Logger {
 public:
  static void set_level(LogLevel lvl) { level_ = lvl; }
  static LogLevel level() { return level_; }

  static void log(LogLevel lvl, const char* tag, const char* fmt, ...) {
    if (static_cast<int>(lvl) < static_cast<int>(level_)) return;
    va_list args;
    va_start(args, fmt);
#if defined(__PROS__) || __has_include("pros/apix.h")
    // On robot, print to serial
    std::printf("[%s] ", tag);
    vprintf(fmt, args);
    std::printf("\n");
#else
    // Host
    std::fprintf(stderr, "[%s] ", tag);
    std::vfprintf(stderr, fmt, args);
    std::fprintf(stderr, "\n");
#endif
    va_end(args);
  }

 private:
  static LogLevel level_;
};

}  // namespace vexlib
