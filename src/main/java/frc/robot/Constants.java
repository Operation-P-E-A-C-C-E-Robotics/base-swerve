// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleFunction;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
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

public final class Constants {
  public static final double period = 0.01;

  public static final class Shooter {
    public static final int upperFlywheelMotorId = 0;
    public static final int lowerFlywheelMotorId = 0;
    public static final int triggerMotorId = 0;

    public static final double flywheelMaxControllableVelocity = 0; //rotations per second
    public static final double flywheelGearRatio = 1;
    public static final double flywheelDiameter = Units.inchesToMeters(5);
    public static final double flywheelKv = 0;
    public static final double flywheelKa = 0;
    public static final double flywheelModelStDev = 0;
    public static final double flywheelEncoderStDev = 0;
    public static final double flywheelControlEffort = 0;
    public static final double flywheelErrorTolerance = 0;

    public static final double flywheelEfficiency = 1; // percentage of flywheel surface speed to exit velocity
    public static final double flywheelTolerance = 0; //how close to the target velocity the flywheel needs to be considered ready

    public static final double triggerMetersPerRotation = 0;
    public static final double triggerKp = 0;
  }

  public static final class FlywheelIntake {
    public static final int flywheelIntakeDeployMotorId = 0;
    public static final int flywheelIntakeRollerMotorId = 0;

    public static final double flywheelIntakeDeployKp = 0;
    public static final double flywheelIntakeDeployKi = 0;
    public static final double flywheelIntakeDeployKd = 0;

    public static final double flywheelIntakeDeployMinAngle = 0;
    public static final double flywheelIntakeDeployMaxAngle = 360;

    public static final double flywheelIntakeDeployGearing = 1;
    public static final double flywheelIntakeDeployTolerance = 0; //how close to the target position the deployer needs to be to be considered "deployed"

    public static final boolean flywheelIntakeDeployMotorInverted = false;
    public static final boolean flywheelIntakeRollerMotorInverted = false;
  }

  public static final class TriggerIntake {
    public static final int triggerIntakeMotorId = 0;
    public static final int triggerIntakeDeployMotorId = 0;

    public static final double triggerIntakeDeployKp = 0;
    public static final double triggerIntakeDeployKi = 0;
    public static final double triggerIntakeDeployKd = 0;

    public static final double triggerIntakeDeployGearing = 1;
    public static final double triggerIntakeDeployTolerance = 0; //how close to the target position the deployer needs to be to be considered "deployed"
  }

  public static final class Pivot {
    public static final int pivotMotorId = 0;

    public static final double pivotMinAngle = 0;
    public static final double pivotMaxAngle = 0;

    public static final double pivotGearRatio = 1;
    public static final double pivotTolerance = 0; //how close to the target position the pivot needs to be considered ready
    
    public static final double pivotKp = 0;
    public static final double pivotKi = 0;
    public static final double pivotKd = 0;
    public static final double pivotKs = 0;
    public static final double pivotKv = 0;
    public static final double pivotKa = 0;
    public static final double pivotCruiseVelocity = 0;
    public static final double pivotMotionMagicExpoKv = pivotKv;
    public static final double pivotMotionMagicExpoKa = pivotKa;
  }

  public static final class Diverter {
    public static final int diverterMotorId = 0;
    public static final int diverterDeployMotorId = 0;

    public static final double diverterDeployGearRatio = 1;
    public static final double diverterDeployTolerance = 0; //how close to the target position the deployer needs to be to be considered "deployed"

    public static final double maxDiverterExtension = 0; //in meters

    public static final double diverterDeployKp = 0;
    public static final double diverterDeployKi = 0;
    public static final double diverterDeployKd = 0;
    public static final double diverterDeployKs = 0;
    public static final double diverterDeployKv = 0;
    public static final double diverterDeployKa = 0;
    public static final double diverterDeployCruiseVelocity = 0;
    public static final double diverterDeployMotionMagicExpoKv = diverterDeployKv;
    public static final double diverterDeployMotionMagicExpoKa = diverterDeployKa;
  }

  public static final class Climber {
    public static final int climberLeftMotorId = 0;
    public static final int climberRightMotorId = 0;

    public static final double climberGearRatio = 1;

    public static final double climberTolerance = 0; //how close to the target position the climber needs to be considered ready

    public static final double climberDeployKp = 0;
    public static final double climberDeployKi = 0;
    public static final double climberDeployKd = 0;
    public static final double climberDeployKs = 0;
    public static final double climberDeployKv = 0;
    public static final double climberDeployKa = 0;
    public static final double climberDeployCruiseVelocity = 0;
    public static final double climberDeployMotionMagicExpoKv = climberDeployKv;
    public static final double climberDeployMotionMagicExpoKa = climberDeployKa;
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
    public static final Dimensions dimensions = new Dimensions(Units.inchesToMeters(24.75), Units.inchesToMeters(24.75));

    //to: whoever did this; why did you label the modules differetly on the robot vs the tuner????? -love, peaccy
    public static final CANIDs frontLeftIDs =   new CANIDs(8,   10,   9); //module 3 (tuner) / 2 (robot)
    public static final CANIDs frontRighIDs =   new CANIDs(5,   7,    6); //module (tuner) 2 / (robot) 1
    public static final CANIDs rearLeftIDs =    new CANIDs(11,  13,   12); //module (tuner) 4 / (robot) 3
    public static final CANIDs rearRightIDs =   new CANIDs(2,   4,    3); //module (tuner) 1 / (robot) 0

    public static final Gearing gearing = new Gearing(DriveGearRatios.SDSMK4i_L2, ((150.0 / 7.0) / 1.0), (3.85/2), 0);
    public static final EncoderOffsets offsets = new EncoderOffsets(-0.488770, -0.225342, -0.224609, -0.906738); //todo these offsets are very wrong.

    public static final Inversion inversion = new Inversion(false, true, true, false); //herra 4.5, 6

    //inertia only used for simulation
    public static final Physics physics = new Physics(0.05,0.01, Robot.isReal() ? 50 : 800, 10);
    public static final double steerMotorCurrentLimit = Robot.isReal() ? 40 : 120; //amps
    
    public static final PidGains driveGains = new PidGains(0.35, 0, 0, 0.11, 0.3); 
    public static final PidGains angleGains = new PidGains(90, 0, 0.001, 0, 0);

    public static final int pigeonCANId = 14;
    public static final boolean invertSteerMotors = Robot.isReal(); //cant invert in simulation which is dumb.

    /* HEADING CONTROLLER CONSTANTS */
    public static final double autoHeadingKP = 2000;//500;
    public static final double autoHeadingKI = 0.0; //DOES NOTHING LOL
    public static final double autoHeadingKD = 0.0; //ALSO DOES NOTHING LOL
    public static final double autoHeadingKV = 10;//0.7;
    public static final double autoHeadingKA = 0.02;
    public static final double autoHeadingMaxVelocity = 3; //deg/s (i think)
    public static final double autoHeadingMaxAcceleration = 10; //deg/s^2
    public static final double lockHeadingMaxVelocity = 6;
    public static final double lockHeadingMaxAcceleration = 20;
    public static final boolean useSoftHoldHeading = false;
    public static final double softHeadingCurrentLimit = 30;

    
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
      new PIDConstants(10, 0, 0), 
      new PIDConstants(5, 0, 0), 
      pathfollowingMaxVelocity, 
      dimensions.frontLeft.getNorm(), //drive radius
      new ReplanningConfig(true, true),
      period
    );

    public static final String primaryLLName = "limelight";
  }

  public static final class ControlSystem {
    public static final int PDPCanId = 0;
    public static final ModuleType PDPModuleType = ModuleType.kCTRE;
  }

  public static final class Field{
    public static final Translation2d targetTranslation = new Translation2d(0, 0);
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
