# VEX Robotics Library (vexlib) — PROS Starter

A modern, extensible C++ library for VEX V5 on **PROS** with a clean hardware abstraction layer, controls, and utilities.
This scaffold is designed to work in VS Code with the PROS extension.

> Version: v0.1 (scaffold) — ready to compile and run, with room to grow into a full library.

---

## Features (scaffolded)

- **HAL wrappers** for common V5 devices (Motors, IMU, Distance, Optical, Rotation, GPS, ADI bumper/limit/line).
- **Chassis abstraction** (tank, X-drive ready) with non-blocking arcade/curvature helpers.
- **Controls**: simple **PID** class ready for motion control; ring buffer for filtering.
- **Localization**: 2-wheel odometry (skeleton).
- **Path following**: Pure Pursuit (skeleton).
- **Controller mapping**: high-level helpers for joystick -> chassis.
- **Logger**: minimal, level-based with printf-style formatting.
- Builds on brain (**PROS**) and also builds **host-side** (no PROS) for algorithm tests with CMake/GTest (skeleton).

## Quick Start (Robot)

1. **Install VS Code** + **PROS** extension.
2. Create a new PROS project (or open an existing one).
3. Copy the `include/` and `src/` folders from this repo into your project root (merge if needed).
4. Replace your `src/main.cpp` with the one from this repo (or integrate the parts you like).
5. Build & Upload via PROS: **`pros build`** then **`pros upload`** (or use VS Code buttons).

> PROS project layout normally has `src/` and `include/` next to `project.pros`. This scaffold matches that.

## Quick Start (Host Tests)

1. Install **CMake** and a C++17 compiler.
2. From the repo root:
   ```bash
   cmake -S . -B build
   cmake --build build
   ctest --test-dir build
