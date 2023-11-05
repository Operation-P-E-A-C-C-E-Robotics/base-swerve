# base-swerve-2024 
## CTRE Phoenix 6 base swerve code
This is FRC team 3461's possible base robot code for the 2024 season.
- Based on the new CTRE swerve framework
- Field relative and robot relative control
- Automatic heading control to keep robot facing cardinal directions
- Odometry and 2024 pathplanner support
- Toggleable X-locked braking
- Nice joystick smoothing to make it more precise on tiny gamepad joysticks(needs tuning)
This project only works with WPILib 2024 beta, and with all CTRE devices running phoenix 6 firmware.
Easy to set up, just update constants.
# default button mapping
Designed for xbox style controller, axes may vary
- Left stick: heading control
- Right stick: linear direction/velocity control
- Left bumper: hold for robot cetric
- Right bumper: hold to X-lock wheels while stopped
- POV hat: auto rotate to heading
- button 9: zero odometry
- button 5: enable closed loop control
