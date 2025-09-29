# vexlib — Advanced VEX Robotics Library

`vexlib` is a competition-ready robotics library for VEX robots, designed to make autonomous and driver-control code **clean, powerful, and simple**.  

Features include:
- ✅ PID++ (settling, anti-windup, slew, filters, telemetry)
- ✅ Feedforward (kS, kV, kA) with auto-fitting
- ✅ Motion profiles (trapezoid + S-curve)
- ✅ Trajectory follower with events
- ✅ Odometry++ (3-wheel + IMU fusion + GPS correction)
- ✅ Pose math (SE(2), mirroring, transforms)
- ✅ Telemetry (ring buffer, CSV logging to SD)
- ✅ Hardware support (pneumatics, vision pipelines)

---

## 🚀 Installation
1. Clone the repo into your PROS project:
   ```bash
   git clone https://github.com/yourname/vexlib-pros.git include/vexlib
