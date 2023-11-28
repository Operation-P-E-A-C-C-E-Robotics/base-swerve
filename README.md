# base-swerve-2024 
## CTRE Phoenix 6 base swerve code
This is FRC team 3461's base robot code for the 2024 season - currently (permanently) under development.
- Based on the new CTRE swerve framework
- Field relative and robot relative control
- Automatic heading control to keep robot facing cardinal directions and recover from bumps and drift
- Odometry and 2024 pathplanner
- X-locked braking
- Advanced and highly configurable input curves, smoothing, and deadbanding.
- Extensive telemetry designed to work with advantagescope (no advantagekit because of ctre's multithreading)
Untested / Future features
- 3d odometry with IMU and limelight apriltags
- Teleop position error correction for more consistent feel
- Current-limited auto heading
- Automated test to identify failing modules
This project only works with WPILib 2024 beta, and with all CTRE devices running phoenix 6 firmware.
Easy to set up, just update constants.
# default button mapping ~ will change
Designed for xbox style controller, axes may vary
- Left stick: heading control
- Right stick: linear direction/velocity control
- Left trigger: hold for robot cetric
- Right trigger: hold to X-lock wheels while stopped
- POV hat: auto rotate to heading
- button 9: zero odometry
- button 5: enable closed loop control
