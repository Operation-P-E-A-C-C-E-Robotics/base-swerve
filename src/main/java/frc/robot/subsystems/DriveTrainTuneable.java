package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.RealSwerveDrivetrain;
import frc.lib.swerve.SwerveDescription;
import frc.lib.swerve.SwerveDescription.PidGains;

import static frc.robot.Constants.Swerve.*;

public class DriveTrainTuneable extends SubsystemBase {
    private RealSwerveDrivetrain swerve;

    private final Field2d field = new Field2d();

    private final Joystick driverController = new Joystick(0);

    double linearSpeedDeadband = 0.02,
           angularVelocityDeadband = 0.02;

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.FieldCentricFacingAngle autoHeadingRequest = new SwerveRequest.FieldCentricFacingAngle();
    private final SwerveRequest.SwerveDriveBrake lockInRequest = new SwerveRequest.SwerveDriveBrake();

    private double newAutoHeadingKP = autoHeadingKP;
    private double newAutoHeadingKI = autoHeadingKI;
    private double newAutoHeadingKD = autoHeadingKD;
    private double newTeleopLinearMultiplier = teleopLinearMultiplier;
    private double newTeleopAngularMultiplier = teleopAngularMultiplier;

    private SlewRateLimiter linearSpeedLimiter = new SlewRateLimiter(teleopLinearSpeedLimit);
    private SlewRateLimiter linearAngleLimiter = new SlewRateLimiter(teleopLinearAngleLimit);
    private SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(teleopAngularRateLimit);


    /**
     * DO NOT USE THIS INSTEAD OF DriveTrain.java
     * HAS BUILT IN DRIVE MODE SO DOESNT WORK WITH COMMANDS
     * This is a copy of DriveTrain.java that is used for tuning the drivetrain.
     * It is not used in the actual robot code.
     * It adds a bunch of SmartDashboard stuff to make it easier to tune the drivetrain.
     * I'm sure it will end up being a hot mess - Sean (before writing this hot mess - Sean (after writing this hot mess))
     */
    public DriveTrainTuneable() {
        swerve = SwerveDescription.generateDrivetrain(
            dimensions, 
            frontLeftIDs, 
            frontRighIDs, 
            rearLeftIDs, 
            rearRightIDs, 
            gearing, 
            offsets, 
            inversion, 
            physics, 
            driveGains, 
            angleGains, 
            pigeonCANId, 
            invertSteerMotors
        ); 

        //log swerve state data as fast as it comes in
        swerve.registerTelemetry((SwerveDriveState state) -> {
            field.setRobotPose(state.Pose);
            SmartDashboard.putNumber("Front Left Module Angle", state.ModuleStates[0].angle.getDegrees());
            SmartDashboard.putNumber("Front Right Module Angle", state.ModuleStates[1].angle.getDegrees());
            SmartDashboard.putNumber("Rear Left Module Angle", state.ModuleStates[2].angle.getDegrees());
            SmartDashboard.putNumber("Rear Right Module Angle", state.ModuleStates[3].angle.getDegrees());
            SmartDashboard.putNumber("Front Left Module Speed", state.ModuleStates[0].speedMetersPerSecond);
            SmartDashboard.putNumber("Front Right Module Speed", state.ModuleStates[1].speedMetersPerSecond);
            SmartDashboard.putNumber("Rear Left Module Speed", state.ModuleStates[2].speedMetersPerSecond);
            SmartDashboard.putNumber("Rear Right Module Speed", state.ModuleStates[3].speedMetersPerSecond);
        });

        SmartDashboard.putNumber("drive kp", driveGains.kP);
        SmartDashboard.putNumber("drive ki", driveGains.kI);
        SmartDashboard.putNumber("drive kd", driveGains.kD);
        SmartDashboard.putNumber("drive kv", driveGains.kV);
        SmartDashboard.putNumber("drive ka", driveGains.kA);

        SmartDashboard.putNumber("angle kp", angleGains.kP);
        SmartDashboard.putNumber("angle ki", angleGains.kI);
        SmartDashboard.putNumber("angle kd", angleGains.kD);
        SmartDashboard.putNumber("angle kv", angleGains.kV);
        SmartDashboard.putNumber("angle ka", angleGains.kA);

        SmartDashboard.putNumber("auto heading kp", autoHeadingKP); 
        SmartDashboard.putNumber("auto heading ki", autoHeadingKI);
        SmartDashboard.putNumber("auto heading kd", autoHeadingKD);

        SmartDashboard.putNumber("teleop linear multiplier", teleopLinearMultiplier);
        SmartDashboard.putNumber("teleop angular multiplier", teleopAngularMultiplier);

        SmartDashboard.putNumber("teleop linear speed limit", teleopLinearSpeedLimit);
        SmartDashboard.putNumber("teleop linear angle limit", teleopLinearAngleLimit);
        SmartDashboard.putNumber("teleop angular rate limit", teleopAngularRateLimit);

        SmartDashboard.putNumber("teleop linear deadband", linearSpeedDeadband);
        SmartDashboard.putNumber("teleop angular deadband", angularVelocityDeadband);


        SmartDashboard.putBoolean("update drive", false);


        SmartDashboard.putBoolean("enable tuning drive", false);
        SmartDashboard.putBoolean("field relative", true);
        SmartDashboard.putBoolean("auto heading", false);
        SmartDashboard.putBoolean("lock in", false);
        SmartDashboard.putBoolean("open loop", true);
        SmartDashboard.putBoolean("zero odometry", false);

        SmartDashboard.putNumber("auto heading angle", 0);



        SmartDashboard.putData("Field", field);

        System.out.println("DriveTrain Initialized");
    }

    //re initialize the drivetrain with new gains
    public void updateDrive() {
        var newDriveGains = new PidGains(
            SmartDashboard.getNumber("drive kp", driveGains.kP),
            SmartDashboard.getNumber("drive ki", driveGains.kI),
            SmartDashboard.getNumber("drive kd", driveGains.kD),
            SmartDashboard.getNumber("drive kv", driveGains.kV),
            SmartDashboard.getNumber("drive ka", driveGains.kA)
        );

        var newAngleGains = new PidGains(
            SmartDashboard.getNumber("angle kp", angleGains.kP),
            SmartDashboard.getNumber("angle ki", angleGains.kI),
            SmartDashboard.getNumber("angle kd", angleGains.kD),
            SmartDashboard.getNumber("angle kv", angleGains.kV),
            SmartDashboard.getNumber("angle ka", angleGains.kA)
        );

        this.newAutoHeadingKP = SmartDashboard.getNumber("auto heading kp", autoHeadingKP);
        this.newAutoHeadingKI = SmartDashboard.getNumber("auto heading ki", autoHeadingKI);
        this.newAutoHeadingKD = SmartDashboard.getNumber("auto heading kd", autoHeadingKD);

        autoHeadingRequest.HeadingController.setP(newAutoHeadingKP);
        autoHeadingRequest.HeadingController.setI(newAutoHeadingKI);
        autoHeadingRequest.HeadingController.setD(newAutoHeadingKD);

        this.newTeleopLinearMultiplier = SmartDashboard.getNumber("teleop linear multiplier", teleopLinearMultiplier);
        this.newTeleopAngularMultiplier = SmartDashboard.getNumber("teleop angular multiplier", teleopAngularMultiplier);

        linearSpeedLimiter = new SlewRateLimiter(SmartDashboard.getNumber("teleop linear speed limit", teleopLinearSpeedLimit));
        linearAngleLimiter = new SlewRateLimiter(SmartDashboard.getNumber("teleop linear angle limit", teleopLinearAngleLimit));
        angularVelocityLimiter = new SlewRateLimiter(SmartDashboard.getNumber("teleop angular rate limit", teleopAngularRateLimit));

        linearSpeedDeadband = SmartDashboard.getNumber("teleop linear deadband", 0.02);
        angularVelocityDeadband = SmartDashboard.getNumber("teleop angular deadband", 0.02);

        swerve = SwerveDescription.generateDrivetrain(
            dimensions, 
            frontLeftIDs, 
            frontRighIDs, 
            rearLeftIDs, 
            rearRightIDs, 
            gearing, 
            offsets, 
            inversion, 
            physics, 
            newDriveGains, 
            newAngleGains, 
            pigeonCANId, 
            invertSteerMotors
        ); 
    }

    @Override
    public void periodic() {
        //the whole drive command again.
        if(SmartDashboard.getBoolean("update drive", false)) {
            updateDrive();
            SmartDashboard.putBoolean("update drive", false);
        }

        //only drive if we want to
        if(!SmartDashboard.getBoolean("enable tuning drive", false)) return;

        double xVelocity = driverController.getRawAxis(0);
        double yVelocity = driverController.getRawAxis(1);
        double angularVelocity = driverController.getRawAxis(2);

        double autoHeadingAngle = SmartDashboard.getNumber("auto heading angle", 0);
        boolean isAutoHeading = SmartDashboard.getBoolean("auto heading", false);
        boolean isFieldRelative = SmartDashboard.getBoolean("field relative", true);
        boolean isOpenLoop = SmartDashboard.getBoolean("open loop", true);
        boolean isLockIn = SmartDashboard.getBoolean("lock in", false);


        if(SmartDashboard.getBoolean("zero odometry", false)) {
            swerve.tareEverything();
            SmartDashboard.putBoolean("zero odometry", false);
        }

        // handle smoothing and deadbanding
        Translation2d linearVelocity = new Translation2d(xVelocity, yVelocity);
        linearVelocity = smoothAndDeadband(linearVelocity).times(newTeleopLinearMultiplier);
        angularVelocity = smoothAndDeadband(angularVelocity) * newTeleopAngularMultiplier;

        //handle lock in
        if (isLockIn) {
            if (linearVelocity.equals(new Translation2d(0,0)) && angularVelocity == 0) {
                swerve.setControl(lockInRequest.withIsOpenLoop(isOpenLoop));
                return;
            }
        }

        //handle auto angle
        if (isAutoHeading) {
            autoHeadingRequest.withIsOpenLoop(isOpenLoop)
                            .withVelocityX(linearVelocity.getX())
                            .withVelocityY(linearVelocity.getY())
                            .withTargetDirection(Rotation2d.fromDegrees(autoHeadingAngle))
                            .withRotationalDeadband(0.1); // todo do we need rotational deadband?

            swerve.setControl(autoHeadingRequest);
            return;
        }

        //handle field relative
        if (isFieldRelative) {
            fieldCentricRequest.withIsOpenLoop(isOpenLoop)
                                .withVelocityX(linearVelocity.getX())
                                .withVelocityY(linearVelocity.getY())
                                .withRotationalRate(angularVelocity);

            swerve.setControl(fieldCentricRequest);
            return;
        }

        //handle robot relative
        robotCentricRequest.withIsOpenLoop(isOpenLoop)
                            .withVelocityX(linearVelocity.getX())
                            .withVelocityY(linearVelocity.getY())
                            .withRotationalRate(angularVelocity);

        swerve.setControl(robotCentricRequest);
    }

    
    private Translation2d smoothAndDeadband (Translation2d linearVelocity) {
        //handle deadband and reset the rate limiter if we're in the deadband
        double rawLinearSpeed = handleDeadbandFixSlope(linearSpeedDeadband,linearVelocity.getNorm());
        if(Math.abs(rawLinearSpeed) < linearSpeedDeadband) linearSpeedLimiter.reset(0);

        //smooth the linear speed
        double linearSpeed = linearSpeedLimiter.calculate(rawLinearSpeed);

        //smooth the linear angle
        double rawLinearAngle = linearVelocity.getAngle().getRadians();
        double linearAngle = linearAngleLimiter.calculate(rawLinearAngle);

        // override the smoothing if it lags too far behind the raw value
        // (mainly after stopping and changing direction)
        if (Math.abs(linearAngle - rawLinearAngle) > Math.PI/4) {
            linearAngleLimiter.reset(rawLinearAngle);
            linearAngle = rawLinearAngle;
        }

        return new Translation2d(linearSpeed, new Rotation2d(linearAngle));
    }

    private double smoothAndDeadband (double angularVelocity) {
        //apply deadband to angular velocity
        angularVelocity = handleDeadbandFixSlope(angularVelocityDeadband, angularVelocity);

        //simple square curve on the angular velocity to make it more responsive
        angularVelocity = Math.copySign(Math.pow(angularVelocity, 2), angularVelocity);

        //limit the angular velocity
        angularVelocity = angularVelocityLimiter.calculate(angularVelocity);

        return angularVelocity;
    }

    /**
     * This function handles deadband by increases the slope of the output after
     * the deadband, so the output is 0 at the end of the deadband but still 100% for 100% input. 
     * This preserves fine control while still eliminating unnecessary current draw and preventing joystick drift.
     * @param deadband the deadband to use
     * @param value the value to apply the deadband to
     * @return the value with the deadband applied (like magic)
     */
    private double handleDeadbandFixSlope (double deadband, double value) {
        if (Math.abs(value) < deadband) return 0;
        return (value - (deadband * Math.signum(value)))/(1 - deadband);
    }
    
}


