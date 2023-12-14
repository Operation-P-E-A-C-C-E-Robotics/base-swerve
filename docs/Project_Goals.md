# Project Goals
This is my attemp at creating the best base robot code I am able to before I graduate from the team. This is the culmination of approximately 6 years of frc programming experience. I want to cover as many different use cases as possible, to be adaptable for as many future games as possible. Due to time limitations, I am unable to add many useful libraries for different robot mechanisms besides the swerve drivetrain, but I have included the resource-heavy but very accurate motion profiling and state-space controller libraries from our 2023 robot. (Find them in frc/lib/motion and frc/lib/util). Here is a list of my priorities for this project:
 - Swerve code that is as easy to drive and accurate as possible. Speed is useless if you can't control that speed.
 - Accurate pathfollowing & auton framework (accomplished with PathPlannerLib :D)
 - Apriltag pose estimation with limelight
 - Libraries to accelerate the development process
 - Extremely clear documentation - our new programmer needs something to be able to learn from and reuse.
 - Log lots and lots of debugging data - EASY TO DETECT FAILURES
 - Well-organized and consistent code structure
 - Resillient to as many failure cases as possible

One of my highest priorities in this project is being able to detect failing hardware using the software. There have been countless times where hardware has failed or begun to fail in competition and it goes completely unnoticed for several matches, despite impacting performanve. I believe this can be resolved by 2 things: Logging a hitton of data, and having a rigorous pre-match checklist.