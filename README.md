# Newbies-BFMC2026

This repository contains the BFMC 2026 project workspace, including embedded platform code, calibration data, planning resources, and monitoring utilities.

## Repository structure

- Calibration/` - Contains calibartion process of BNO005 IMU.
- Embedded_Platform/` - Contains embedded system source and platform-specific files.
- monitoring/`  - Contains monitoring and diagnostics tools or scripts.
- Project_status/` - Contains the status of project.
- src/ - Contains a ROS/robotics planning workspace with `CMakeLists.txt` and several packages.
- 
## Notes
- planning_ws/ is a ROS workspace with build artifacts and source packages for path planning, communication, persistence, and utilities.
- The calibration folder includes IMU calibration code for a BNO055 sensor.

## Usage
1. Use the ROS workspace under `src/planning_ws/` for path planning and robotics-related development.
2. Use the `Calibration/IMU_BNO055_starfruit/Code_calib_IMU/` files for IMU calibration procedures.

## License

Add license information here if available.
