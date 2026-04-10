# yeastcppwpilibdrivecontroller - Growth Opportunities

## Improvement Opportunities

### 1. Initialize `module_statuses` to safe defaults in constructor
Pre-fill `module_statuses` with 4 default `SwerveModuleStatus` objects (all zeros) in the constructor, similar to how `module_commands` is initialized at `src/wpilibdrivecontroller.cpp:46-50`. This prevents the UB crash if `drive()` is called before `update_motor_status()`.

### 2. Desaturation of module speeds
The WPILib API provides `SwerveDriveKinematics::DesaturateWheelSpeeds()` which is not called anywhere. Without desaturation, if any module exceeds the physical max speed, the robot's motion direction will be distorted.

### 3. Support configurable module count
Currently hardcoded to 4 modules. Consider making the module count a template parameter or runtime configuration to support 6-module or 8-module swerve designs.

### 4. Return acceleration information from `drive()`
`drive()` at line 109-116 builds a `MotionSample` with `pose_valid = false` but never computes or returns actual acceleration feedback. The acceleration information from the command is just passed through, not derived from the actual module behavior.

## Testing Gaps

### 5. No unit tests
No tests exist for:
- Constructor with valid/invalid JSON configurations
- `drive()` with various velocity commands (zero, straight, rotation-only, combined)
- Module angle optimization correctness
- Behavior when `update_motor_status()` has not been called
- Cosine compensation effects

### 6. No integration test with the odometry provider
The drive controller and odometry provider are tightly coupled through `SwerveModuleCommand`/`SwerveModuleStatus`, but there is no test verifying the round-trip.
