// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.function.DoubleFunction;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.lib.swerve.SwerveDescription.CANIDs;
import frc.lib.swerve.SwerveDescription.Dimensions;
import frc.lib.swerve.SwerveDescription.EncoderOffsets;
import frc.lib.swerve.SwerveDescription.Gearing;
import frc.lib.swerve.SwerveDescription.Inversion;
import frc.lib.swerve.SwerveDescription.Physics;
import frc.lib.swerve.SwerveDescription.PidGains;
import frc.lib.util.JoystickCurves;
import frc.lib.vision.ApriltagCamera.ApriltagPhotonvision;

public final class Constants {
  public static final double period = 0.01;

  public static final class Cameras {
    public static final String exampleLimelight = "limelight";

    public static final String examplePhotonvisionName = "photonvision";

    public static final double LIMELIGHT_FOCAL_LENGTH = (1*83)/0.32; //as calculated by me: (distance * pixels) / size

    public static final Transform3d robotToExamplePhotonvision = new Transform3d(
        Units.inchesToMeters(10.5), //forward offset
        Units.inchesToMeters(6),    
        Units.inchesToMeters(8.5), 
        new Rotation3d(0,Units.degreesToRadians(15),0)
    );

    public static final ApriltagPhotonvision examplePhotonvision = new ApriltagPhotonvision(
      examplePhotonvisionName,
      robotToExamplePhotonvision,
      FieldConstants.aprilTags,
      0.9
    );
  }
  public static final class Swerve {
    /* TELEOP */
    //speeds in m/s (probably)
    public static final double teleopLinearMultiplier = 7.0;
    public static final double teleopAngularMultiplier = 7.0;

    //acceleration limits
    public static final double teleopLinearSpeedLimit = 5.0;
	  public static final double teleopLowBatteryLinearSpeedLimit = 2; //more acceleration limit when battery is low
    public static final double teleopLinearAngleLimit = 2.0;
    public static final double teleopAngularRateLimit = 3.0;

    //deadband
    public static final double teleopLinearSpeedDeadband = 0.15;
    public static final double teleopAngularVelocityDeadband = 0.2;
    public static final double teleopDeadbandDebounceTime = 0.1;

    public static final DoubleFunction <Double> teleopLinearSpeedCurve = (double linearSpeed) -> JoystickCurves.herraFCurve(linearSpeed, 6, 4.5); //a nice gentle curve which is Peaccy's (me!!) favorite :)
    public static final DoubleFunction <Double> teleopAngularVelocityCurve = (double angularVelocity) -> JoystickCurves.powerCurve(angularVelocity, 2); //TODO decide if the driver (me) wants a curve on this or not.

    //number of loops to keep track of position correction for (so multiply by 20ms to get the duration the correction is considering)
    //todo too slow?
    public static final int teleopPositionCorrectionIters = 0; 




    /* CTRE SWERVE CONSTANTS */
    public static final Dimensions dimensions = new Dimensions(Units.inchesToMeters(18.75), Units.inchesToMeters(18.75));

    public static final CANIDs frontLeftIDs =   new CANIDs(4,   5,    6); //module 0
    public static final CANIDs rearLeftIDs =   new CANIDs(7,   8,    9); //module 1
    public static final CANIDs rearRightIDs =   new CANIDs(10,   11,   12); //module 2
    public static final CANIDs frontRightIDs =    new CANIDs(13,  14,   15); //module 3

    public static final Gearing gearing = new Gearing(DriveGearRatios.SDSMK4i_L2, ((150.0 / 7.0) / 1.0), (3.807/2), 0);
    // public static final EncoderOffsets offsets = new EncoderOffsets(-0.488770, -0.225342, -0.224609, -0.906738); //todo these offsets are very wrong.
    public static final EncoderOffsets offsets = new EncoderOffsets(
      -0.409424, //Front Left, module 0
      0.277344, //Front Right, module 3
      -0.412109,  // Rear Left, module 1
      -0.307861); //Rear Right, module 2

    public static final Inversion inversion = new Inversion(false, true, false, true);

    //inertia only used for simulation
    public static final Physics physics = new Physics(0.05,0.01, Robot.isReal() ? 80 : 800, 7);
    public static final double steerMotorCurrentLimit = Robot.isReal() ? 40 : 120; //amps
    
    public static final PidGains driveGains = new PidGains(2, 0, 0, 0.2, 0.4); 
    public static final PidGains angleGains = new PidGains(90, 0, 0.001, 0, 0);

    public static final int pigeonCANId = 3;
    public static final boolean invertSteerMotors = Robot.isReal(); //cant invert in simulation which is dumb.

    /* HEADING CONTROLLER CONSTANTS */
    public static final double autoHeadingKP = 400;
    public static final double lockHeadingKP = 1000;
    public static final double autoHeadingKI = 0.0; //DOES NOTHING LOL
    public static final double autoHeadingKD = 0.0; //ALSO DOES NOTHING LOL
    public static final double autoHeadingKV = 0.7;
    public static final double autoHeadingKA = 0.02;
    public static final double autoHeadingMaxVelocity = 3; //deg/s (i think)
    public static final double autoHeadingMaxAcceleration = 10; //deg/s^2
    public static final double lockHeadingMaxVelocity = 6;
    public static final double lockHeadingMaxAcceleration = 20;
    public static final boolean useSoftHoldHeading = false;
    public static final double softHeadingCurrentLimit = 30;

    public static final double aimTolerance = 1; //degrees

    
    /* PATH FOLLOWING CONSTANTS */
    public static final double pathfollowingMaxVelocity = 3,
                              pathfollowingMaxAcceleration = 3,
                              pathfollowingMaxAngularVelocity = 360,
                              pathfollowingMaxAngularAcceleration = 360;

    public static final double measuredMaxVelocity = 3,
                              measuredMaxAcceleration = 3,
                              measuredMaxAngularVelocity = 360,
                              measuredMaxAngularAcceleration = 360;

    public static final PathConstraints autoMaxSpeed = new PathConstraints(
      pathfollowingMaxVelocity,
      pathfollowingMaxAcceleration,
      pathfollowingMaxAngularVelocity,
      pathfollowingMaxAngularAcceleration
    );

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(3, 0, 0), 
      new PIDConstants(5, 0, 0), 
      pathfollowingMaxVelocity, 
      dimensions.frontLeft.getNorm(), //drive radius
      new ReplanningConfig(true, true),
      period
    );
  }

  public static final class ControlSystem {
    public static final int PDPCanId = 0;
    public static final ModuleType PDPModuleType = ModuleType.kCTRE;
  }

  public static final class FieldConstants {
    public static final AprilTagFieldLayout aprilTags;
    static {
      try {
        aprilTags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }
  }

  //stolen from 364 :D
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
