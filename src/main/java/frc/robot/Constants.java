// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.lib.swerve.SwerveDescription.CANIDs;
import frc.lib.swerve.SwerveDescription.Dimensions;
import frc.lib.swerve.SwerveDescription.EncoderOffsets;
import frc.lib.swerve.SwerveDescription.Gearing;
import frc.lib.swerve.SwerveDescription.Inversion;
import frc.lib.swerve.SwerveDescription.Physics;
import frc.lib.swerve.SwerveDescription.PidGains;

public final class Constants {
  public static final class Swerve {
    //speeds in m/s (probably)
    public static final double teleopLinearMultiplier = 7.0;
    public static final double teleopAngularMultiplier = 7.0;

    //smoothing
    public static final double teleopLinearSpeedLimit = 5.0;
    public static final double teleopLinearAngleLimit = 2.0;
    public static final double teleopAngularRateLimit = 3.0;

    //CTRE SWERVE CONSTANTS:
    public static final Dimensions dimensions = new Dimensions(Units.inchesToMeters(24.75), Units.inchesToMeters(24.75));

    //to: whoever did this; why did you label the modules differetly on the robot vs the tuner????? -love, sean
    public static final CANIDs frontLeftIDs =   new CANIDs(8,   10,   9); //module 3 (tuner) / 2 (robot)
    public static final CANIDs frontRighIDs =   new CANIDs(5,   7,    6); //module (tuner) 2 / (robot) 1
    public static final CANIDs rearLeftIDs =    new CANIDs(11,  13,   12); //module (tuner) 4 / (robot) 3
    public static final CANIDs rearRightIDs =   new CANIDs(2,   4,    3); //module (tuner) 1 / (robot) 0

    public static final Gearing gearing = new Gearing(DriveGearRatios.SDSMK4i_L2, ((150.0 / 7.0) / 1.0), 2.0, 0);
    public static final EncoderOffsets offsets = new EncoderOffsets(-0, -0.227539, -0.708496, -0.407959); //todo these offsets are very wrong.

    public static final Inversion inversion = new Inversion(true, true, false, true);

    //inertia only used for simulation, which doesn't seem to work regardless. tf ctre.
    public static final Physics physics = new Physics(0.01, 50, 40, 10);
    
    //unknown units. tf ctre.
    public static final PidGains driveGains = new PidGains(0.2, 0, 0, 0, 0); 
    public static final PidGains angleGains = new PidGains(70, 0, 0, 0, 0);
  
    public static final int pigeonCANId = 14;
    public static final boolean invertSteerMotors = true;

    public static final double autoHeadingKP = 2.0;
    public static final double autoHeadingKI = 0.0;
    public static final double autoHeadingKD = 0.0;

    //TODO
    public static final double measuredMaxVelocity = 1.0,
                              measuredMaxAcceleration = 1.0,
                              measuredMaxAngularVelocity = 1.0,
                              measuredMaxAngularAcceleration = 1.0;
    
    public static final double autoMaxSpeedSafetyScalar = 0.9;

    public static final PathConstraints autoMaxSpeed = new PathConstraints(
      measuredMaxVelocity * autoMaxSpeedSafetyScalar, 
      measuredMaxAcceleration * autoMaxSpeedSafetyScalar, 
      measuredMaxAngularVelocity * autoMaxSpeedSafetyScalar, 
      measuredMaxAngularAcceleration * autoMaxSpeedSafetyScalar
    );

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(10, 0, 0), 
      new PIDConstants(10, 0, 0), 
      measuredMaxVelocity, 
      dimensions.frontLeft.getNorm(), 
      new ReplanningConfig(false, false),
      0.02
    );
  }

  public static final class Core {
    public static final int PDPCanId = 0;
    public static final ModuleType PDPModuleType = ModuleType.kCTRE;
  }

  public class DriveGearRatios{
    /** SDS MK3 - 8.16 : 1 */
    public static final double SDSMK3_Standard = (8.16 / 1.0);
    /** SDS MK3 - 6.86 : 1 */
    public static final double SDSMK3_Fast = (6.86 / 1.0);

    /** SDS MK4 - 8.14 : 1 */
    public static final double SDSMK4_L1 = (8.14 / 1.0);
    /** SDS MK4 - 6.75 : 1 */
    public static final double SDSMK4_L2 = (6.75 / 1.0);
    /** SDS MK4 - 6.12 : 1 */
    public static final double SDSMK4_L3 = (6.12 / 1.0);
    /** SDS MK4 - 5.14 : 1 */
    public static final double SDSMK4_L4 = (5.14 / 1.0);
    
    /** SDS MK4i - 8.14 : 1 */
    public static final double SDSMK4i_L1 = (8.14 / 1.0);
    /** SDS MK4i - 6.75 : 1 */
    public static final double SDSMK4i_L2 = (6.75 / 1.0);
    /** SDS MK4i - 6.12 : 1 */
    public static final double SDSMK4i_L3 = (6.12 / 1.0);
}
}
