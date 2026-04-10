# yeastcppwpilibdrivecontroller - Issues

## Bugs / Correctness Problems

### 1. `drive()` accesses `module_statuses` before `update_motor_status()` is ever called
In `src/wpilibdrivecontroller.cpp:68-74` and lines 80-83, `drive()` indexes into `module_statuses[0..3]` unconditionally. If `drive()` is called before `update_motor_status()`, `module_statuses` is an empty vector, causing out-of-bounds access and undefined behavior (likely a crash).

### 2. Hardcoded 4-module assumption throughout
The class template parameter is hardcoded to `SwerveDriveKinematics<4>` (`wpilibdrivecontroller.hpp:28`) and all loop indices are hardcoded to 4 (`src/wpilibdrivecontroller.cpp:46,100`). The constructor validates `module_configs.size() != 4` (line 29), but should this support be expanded, the hardcoded constants create fragility.

### 3. Acceleration calculation has division-by-zero potential
In `src/wpilibdrivecontroller.cpp:104-106`, `(optimized_modules[i].speed.value() / total_speed) * total_acceleration` divides by `total_speed`. The guard `has_translation` uses `> 0.0001f` (line 98), but `total_speed` could theoretically be in the range `(0, 0.0001]` through a code path that does not set `has_translation = false` -- however this is actually safe due to the ternary. The real issue is the magic number `0.0001` which has no documented units or rationale.

### 4. `drive()` does not check `command.velocity_valid` or `command.acceleration_valid`
In `src/wpilibdrivecontroller.cpp:53-59`, the method unconditionally reads `command.velocity` and `command.acceleration` regardless of whether `velocity_valid` or `acceleration_valid` are true. Invalid data should not be processed.

## Code Smells

### 5. Unused includes in header
`wpilibdrivecontroller.hpp` includes `<iostream>` and `<nlohmann/json.hpp>` (via component headers) but the header itself does not use them directly.

### 6. Multiple unused includes in the .cpp
`src/wpilibdrivecontroller.cpp` includes `SwerveDrivePoseEstimator.h`, `SwerveDriveOdometry.h`, `SwerveModulePosition.h`, `ChassisSpeeds.h`, `Pose2d.h` -- several of which are not needed by the drive controller (the estimator/odometry headers are for the odometry provider, not this controller).

### 7. `GLOB_RECURSE` in CMakeLists.txt picks up build directory files
`CMakeLists.txt:17-20` uses `file(GLOB_RECURSE SOURCES ...)` which can include generated files from `build/`.

### 8. CMakeLists.txt uses relative path for yeastcpp include
`CMakeLists.txt:39-41` uses `../yeastcpp/include` which is fragile and assumes a specific directory layout. Should use `find_package` or a configurable path.
