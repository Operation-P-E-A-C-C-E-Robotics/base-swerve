# Setting up your driver station
The driver station needs to be treated very carefully. Any issue with the driver station connection to the field makes you unable to compete. Thanks to CSAs and other field staff, this is pretty rare, but to be successful we need to ***Optimize Everything***. Any match where we can't perform removes us from high-level play. Additionally, causing delays on the field trying to sort out issues makes us look unfavorable for alliance selections.

**ALL FRC-specific software gets major updates for each season**, so everything **absolutely must** be upgraded at least once after kickoff. It's also smart to update before each competition. Do it around week in advance though, to make sure you can fix anything the update broke.

## Installing Software
The required software is:
- [The FRC Game Tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html)
- [WPILib & VS Code](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)

Software I reccommend you install is:
- [Phoenix Tuner X](https://apps.microsoft.com/detail/9NVV4PWDW27Z?hl=en-us&gl=US)
- [Rev Hardware Client](https://docs.revrobotics.com/rev-hardware-client/gs/install)
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope/releases/)
- [PathPlanner](https://github.com/mjansen4857/pathplanner/releases)
- [Choreo](https://github.com/SleipnirGroup/Choreo/releases)

Also, *uninstall all unnecessary software*

For most things, you can simply run the new installer to update without having to uninstall the old version. The exeption to this is updating the FRC Game Tools a major version (e.g. 2024 to 2025) - details are provided in the link. 

When you run the WPILib installer for a new season, it will install the new version alongside the old version. The year is part of the program name for all the different tools installed. **Make sure you run the correct version of different tools (e.g. "2025 WPILib VS Code") or you'll run into a lot of issues.**

## Making Sure Everything Always Works
***Please carefully read and follow [WPILibs driver station best practice guide.](https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station-best-practices.html)***

The most important things are:
1. Make sure windows firewall is off
2. Force windows to update around a week before competition.
3. Make sure windows update is off during competitions
4. Make sure power settings don't automatically put the computer to sleep

## How to Use the Driver Station
***PLEASE Read the [WPILib docs on it yo](https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station.html)***

Most important takeaways:
- Team number must be set correctly in the driver station (to match the number programmed into the radio)
- Joysticks must be in the correct slots in the driver station, as defined in the software (which YOU will be writing :D)
- Space bar is *E-Stop*, which is like disabling the robot except much more annoying because to reset it you either need to
    - turn the robot off & on again (non-ideal) OR
    - Press the physical reset button on the RoboRIO (so you don't have to wait for the radio to reboot)

I've seen a bug where the robot E-Stops every time you enable it (which is infuriating). If that happens, try closing and reopening the driver station program, or restarting the computer.


## 0<<---[1][--->>2](02RobotProject.md)
# [INDEX](/README.md)
1. [Setting up your driver station](/GUIDE/01DriverStation.md)
2. [Setting up your robot project](/GUIDE/02RobotProject.md)
3. [Basic programming skills](/GUIDE/03ProgrammingSkills.md)
4. [Using GIT & GitHub](/GUIDE/04GIT&Github.md)
5. [Getting a swerve running (using this project)](/GUIDE/05Swerve.md)
6. [End Effectors, Intakes, and Pneumatics](/GUIDE/06BasicMechs.md)
7. [Motion control 101](/GUIDE/07MotionControl101.md)
8. [How to program an elevator](/GUIDE/08Elevator.md)
9. [How to program an arm](/GUIDE/09Arm.md)
10. [How to program a shooter](/GUIDE/10Shooter.md)
11. [Kinematics 101](/GUIDE/11Kinematics101.md)
12. [Creating good OIs & automation](/GUIDE/12Automation&OI.md)
13. [Creating an autonomous](/GUIDE/13Autonomous.md)
14. [Telemetry & Useful Data Sources](/GUIDE/14Telemetry&Data.md)
15. [Programming Using State Machines](/GUIDE/15StateMachines.md)
16. [Available debugging resources.](/GUIDE/16Debugging.md)
17. [Competitions and Sanity](/GUIDE/17Competition.md)