#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

namespace vexlib {

constexpr const char* VERSION = "0.1.0";

enum class LogLevel : uint8_t { kTrace, kDebug, kInfo, kWarn, kError };

}  // namespace vexlib
