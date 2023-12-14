# base-swerve-2024 
## CTRE Phoenix 6 base swerve code
This is FRC team 3461's base robot code for the 2024 season - currently (permanently) under development.
See the [docs](docs/Outline.md) for more info.
- Based on the new [CTRE swerve framework](https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/swerve/swerve-overview.html)
- Field relative and robot relative control
- Automatic heading control to keep robot facing cardinal directions and recover from bumps and drift
- Odometry and 2024 pathplanner
- X-locked braking
- Advanced and highly [configurable](src/main/java/frc/robot/subsystems/DriveTrainTuner.java) input [curves](src/main/java/frc/lib/util/JoystickCurves.java), smoothing, and deadbanding ([Drive Command](src/main/java/frc/robot/commands/PeaccyDrive.java))
- Extensive [telemetry](src/main/java/frc/lib/telemetry/SwerveTelemetry.java) designed to work with AdvantageScope (no AdvantageKit because CTRE uses multithreading for odometry)
- Teleop position error correction for more consistent feel
- Current-limited auto heading to avoid excessive power consumption and tread wear
### Untested / Future features
- 3d odometry with IMU and limelight apriltags
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
