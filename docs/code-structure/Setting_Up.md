# Setting Up A New (Swerve) Robot!!
#### Warning: The swerve libs are based off of CTRE's Libraries, which only support TalonFX-based motors and CANCoders with a Pigeon 2.
## Basic Configuration
The first things to do when setting up a new bot is to update the CAN IDs, Motor Offsets, Inversions, etc. in the [Constants file](/src/main/java/frc/robot/Constants.java)
1. Set your CAN IDs to something logical in phoenix tuner.
2. Go into constants and find the lines that say "public static final CANIDs". Edit the constructor with your values. The order is "new CANIDs(\<drive motor id\>, \<steer motor id\>, \<CANCoder id\>);
3. enter the pigeon's can ID into the pigeonCANId constant
4. Measure the width and length of your robot and put those values into the Dimensions constructor
5. Enter your module's gear ratios into the Gearing constructor. There are predefined ratios for different SDS module drive ratios. Don't worry about the steer coupling ratio, it can stay 0.
6. Use a piece of extrusion that fits under your drivetrain to hold the wheels pointed perfectly forward. Use phoenix tuner to get the RAW absolute sensor position for each cancoder (WITHOUT the set offset). Enter these values NEGATED into the EncoderOffsets constructor
7. If your modules arent Inverted modules like MK4i, then set invertSteerMotors to false. It is set to Robot.isReal() by default, in order to be "true" for our inverted modules, but "false" when the robot is in simulation. This is because inverted motors break the simulation.
8. Reduce the P gain in angleGains to around 20-30 to ensure nothing breaks. These values will need to be tuned for your robot later, for good performance. Also reduce the steerMotorCurrentLimit to 15-20. We don't want any broken gears!
9. Ensure in [RobotContainer.java](/src/main/java/frc/robot/RobotContainer.java) that the line that says isOpenLoop has true in it's parameter. We don't want to use closed loop controls with untuned controllers,
10. Also check Constants.Core and make sure the PDP can ID and type match your robot, so data logging of the PDP will work.
11. Use trial and error to find motor inversions. Put the robot on blocks, deploy your code, drive forward and see which drive motors rotate backwards. Change those motor's inversion in the Inversion constructor.

Now your swerve drive should be fully functional, but it's tuned for our test chassis. That's no bueno.

## Tuning Velocity Controllers
1. Time to break out the most dreaded file of this project: [DriveTrainTuner.java](/src/main/java/frc/robot/subsystems/DriveTrainTuner.java). Go to [RobotContainer.java](/src/main/java/frc/robot/RobotContainer.java) and comment out the line that declares the normal DriveTrain Subsystem, and the line that declares the PeaccyDrive command, and all the lines that depend on them. Then, uncomment the line with DriveTrainTuneable. Deploy the code, and a whole bunch of crazy values will appear on your ShuffleBoard.
2. Create a new ShuffleBoard tab to escape autopopulation, and drag in the following SmartDashboard values: drive kp, drive ki, drive kd, drive kv, drive ka, angle kp, angle, ki, angle kd, angle kv, and angle ka. Also bring in drive motor errors, and angle motor errors, and set them to show as a graph. finally, bring in "enable tuning drive", "open loop", and "update drive", and show them as boolean buttons.
3. To use the tuner, enable the tuning drive by clicking the button. You can then drive the robot. To change the constants, edit them and then click the "update drive" button to update the motors with the new constants. Watch the graphs to see how well the controllers track errors.
4. To tune the drive controllers, switch the robot to use closed loop control by setting "open loop" to false.
5. Start by changing all the drive constants to zero, and click update drive. the robot shouldn't move when you move the joystick, but you should see the error graph updating.
6. Start with kV. Increase it untill the graph stabilizes at zero while driving the robot at a constant speed. back it down if the graph goes positive without the robot decelrating.
7. Then do kA. This should make the robot track changes in velocity better, since it adds on output power for desired acceleration. Try to tune it so that the robot tracks changes as best it can, but without overshooting.
8. Finally, tune kP. Increase it until your error graph becomes wobbly, or you see overshoot, then decrease it slightly.
9. Copy all the values into your Constants file - they are not saved.


## Drive mode tuning
There are a variety of other tunable parameters in the tuner described above. Here is a list:
- Drive gains (described above)
- Angle motor gains
- Heading gains (tuner for this broken at the time of writing - don't use)
- linear & angle multipliers for teleop (should reflect max speed of robot in m/s)
- acceleration limiters - these make it feel more consistent and reduce wear & battery usage
  - linear speed limit - limits how fast the robot acceleraties in a given direction
  - linear angle limit - limits how fast the robot changes which way it's going
  - angular rate limits - limits the angular acceleration of the robot
- curve sensitivity (angular / linear) controls how aggressive different curve modes work
- linear/angular curve chooser - the curve to apply for linear and angular joystick respectively. The linear curve applies to magnitude of the linear joystick's position, not X/Y separately, since that would make it feel super weird.

The tuner also keeps track of peak robot acceleration and velocity.

*All these values have counterparts in constants that must be edited.*