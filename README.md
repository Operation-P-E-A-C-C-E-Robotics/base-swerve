# ROBOT PROGRAMMING 101
## By Sean Benham
Here is as good a comprehensive guide to programming a robot as I am able to form with my 8 years of FRC knowledge. This is intended to take you from the very basics all the way to a highly competitive robot.

***In many places, I will refer to outside sources.*** This way, I'm not re-inventing the wheel, and all the necessary information is compiled in one place. You won't miss anything for lack of searching. I'll also include the most important tips and tricks that are often missed in conjuction with what's available in whatever artice.

Let's start with that now

## Links to key documentation:
1. [WPILib Docs](https://docs.wpilib.org/en/stable/index.html): Great documentation for all things robot code
2. [Phoenix Docs](https://v6.docs.ctr-electronics.com/en/stable/): Writing software for CTRE products (Motors, Pigeon IMU, CANCoders, etc.)
3. [REV Robotics Docs](https://docs.revrobotics.com/ion-control-system/sw/revlib): Writing software for REV Products (PDH/PCM, Motors, Etc.)
4. [Limelight Docs](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary): Using the Limelight camera
5. [Photonvision Docs](https://docs.photonvision.org/en/latest/): Using photonvision cameras
6. [PathPlanner Docs](https://pathplanner.dev/home.html): Path planning for autos with PathPlanner
7. [Choreo Docs](https://sleipnirgroup.github.io/Choreo/): A more optimized but harder-to-use alternative to PathPlanner
8. ***[EVERYTHING I'VE LEARNED ABOUT FRC (mostly)](https://docs.google.com/document/d/1fm2T6eL4VdnDw-0bCEbM68ifCgFhKDjI2-HoGgEnTO8/edit#heading=h.3j4zhj3pgt6w)***

## Good Reference Projects:
1. [Our 2023 Software](https://github.com/Operation-P-E-A-C-C-E-Robotics/frc-2023) Demonstrates state space motion control with advanced kinematics and automation. Uses limelight apriltags for localizing. Tank drive. Overcomplicated and failed miserably, but structurally sound and worth looking at.
2. [Our 2024 Software](https://github.com/Operation-P-E-A-C-C-E-Robotics/frc-2024) Uses MotionMagicExpo for motion profiling, state space for the flywheels. Swerve drivetrain based on this project. Auto aiming with shoot-on-the-move calculations. Worked extremely well (considering it was written by one guy) but uses an unconventional State Machine architecture. This will make it confusing if you don't have a good understanding of programming in general.
3. [CTRE example projects](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java) Projects from CTRE that demonstrate how to use their stuff. Most of them are not command-based - keep in mind that they aren't examples of how to structure your code.
4. [FRC 254 2018](https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/Robot.java) Very complicated, unconventional, ahead of it's time, and very confusingly structured. This project inspired me so I'm including it, but DON'T try and copy it.
5. [Our 2017 Code](https://github.com/nateblinux/OperationPeacce2017Code/tree/master/src/org/usfirst/frc/team3461/robot) From a time before I did software; much, much simpler than any of the garbage I've done. Don't do a separate subsystem for the left and right half of a tank drive though, that's silly.
6. [Our 2022 Code](https://github.com/Operation-P-E-A-C-C-E-Robotics/frc-2022/blob/main/src/main/java/frc/robot/RobotContainer.java) The first robot I programmed that actually competed (ahem, COVID). Garbage code, I didn't know what I was doing. Worked good.

## Before we get started, look through [this google slide](https://docs.google.com/presentation/d/1DMgxi79YGgkeb51lSAVAxEkoKkov6m2Ys2Yr7qAGM9I/edit?usp=sharing) 
It should give the requisite knowledge of robot hardware to get started. I'm sorry about how terrible it is, that's cause I made it.

# WITHOUT FURTHER ADO
## Here is the guide index:
1. [Setting up your driver station](GUIDE/01DriverStation.md)
2. [Setting up your robot project](GUIDE/02RobotProject.md)
3. [Basic programming skills](GUIDE/03ProgrammingSkills.md)
4. [Using GIT & GitHub](GUIDE/04GIT&Github.md)
5. [Getting a swerve running (using this project)](GUIDE/05Swerve.md)
6. [End Effectors, Intakes, and Pneumatics](GUIDE/06BasicMechs.md)
7. [Motion control 101](GUIDE/07MotionControl101.md)
8. [How to program an elevator](GUIDE/08Elevator.md)
9. [How to program an arm](GUIDE/09Arm.md)
10. [How to program a shooter](GUIDE/10Shooter.md)
11. [Kinematics 101](GUIDE/11Kinematics101.md)
12. [Creating good OIs & automation](GUIDE/12Automation&OI.md)
13. [Creating an autonomous](GUIDE/13Autonomous.md)
14. [Telemetry & Useful Data Sources](GUIDE/14Telemetry&Data.md)
15. [Programming Using State Machines](GUIDE/15StateMachines.md)
16. [Available debugging resources.](GUIDE/16Debugging.md)
17. [Competitions and Sanity](GUIDE/17Competition.md)