package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.PeaccefulSwerve;
import frc.lib.swerve.SwerveDescription;
import frc.lib.swerve.SwerveDescription.PidGains;
import frc.lib.telemetry.SwerveTelemetry;
import frc.lib.util.JoystickCurves;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.*;

public class DriveTrainTuner extends SubsystemBase {
    private PeaccefulSwerve swerve;

    private final Joystick driverController = new Joystick(0);

    double linearSpeedDeadband = 0.1,
           angularVelocityDeadband = 0.1;

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

    private double linearCurveSensitivity = 6;
    private double angularCurveSensitivity = 6;

    private SendableChooser<JoystickCurves.CurveType> linearCurveChooser = new SendableChooser<>();
    private SendableChooser<JoystickCurves.CurveType> angularCurveChooser = new SendableChooser<>();

    private double maxLinearSpeed = 0;
    private double maxLinearAcceleration = 0;
    private double maxAngularVelocity = 0;
    private double maxAngularAcceleration = 0;

    private Translation2d lastChassisSpeed = new Translation2d(0,0);

    private double lastRotationalSpeed;

    /**
     * DO NOT USE THIS INSTEAD OF DriveTrain.java
     * HAS BUILT IN DRIVE MODE SO DOESNT WORK WITH COMMANDS
     * This is a copy of DriveTrain.java that is used for tuning the drivetrain.
     * It is not used in the actual robot code.
     * It adds a bunch of SmartDashboard stuff to make it easier to tune the drivetrain.
     * I'm sure it will end up being a hot mess - Peaccy (before writing this hot mess - Peaccy (after writing this hot mess))
     * IMPORTANT!!: THE ONLY REASON I'M ALLOWING THIS DUMPSTER FIRE TO EXIST IS BECAUSE IT WILL NEVER EVER EVER BE RUN AT COMPETITION. RIGHT???
     * I say run at competition above, however i have come to the conclusion that it will be a miracle if it ever runs at all. - Peaccy
     */
    public DriveTrainTuner() {
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
            SwerveTelemetry.updateSwerveState(state, swerve.getChassisSpeeds());
            SmartDashboard.putNumber("Front Left Module Angle", state.ModuleStates[0].angle.getRotations());
            SmartDashboard.putNumber("Front Right Module Angle", state.ModuleStates[1].angle.getRotations());
            SmartDashboard.putNumber("Rear Left Module Angle", state.ModuleStates[2].angle.getRotations());
            SmartDashboard.putNumber("Rear Right Module Angle", state.ModuleStates[3].angle.getRotations());
            SmartDashboard.putNumber("Front Left Module Speed", state.ModuleStates[0].speedMetersPerSecond);
            SmartDashboard.putNumber("Front Right Module Speed", state.ModuleStates[1].speedMetersPerSecond);
            SmartDashboard.putNumber("Rear Left Module Speed", state.ModuleStates[2].speedMetersPerSecond);
            SmartDashboard.putNumber("Rear Right Module Speed", state.ModuleStates[3].speedMetersPerSecond);
        });

        //log current gains
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

        SmartDashboard.putNumber("linear curve sensitivity", linearCurveSensitivity);
        SmartDashboard.putNumber("angular curve sensitivity", angularCurveSensitivity);


        //different input curves to play with
        linearCurveChooser.addOption("Linear", JoystickCurves.CurveType.LINEAR);
        linearCurveChooser.addOption("Power", JoystickCurves.CurveType.POWER);
        linearCurveChooser.addOption("Exponential", JoystickCurves.CurveType.EXPONENTIAL);
        linearCurveChooser.addOption("FS2 Default", JoystickCurves.CurveType.FS2_DEFAULT);
        linearCurveChooser.addOption("FS2 Windows", JoystickCurves.CurveType.FS2_WINDOWS);
        linearCurveChooser.setDefaultOption("Herra 4.5 F Curve", JoystickCurves.CurveType.HERRA4_5_F_CURVE);
        linearCurveChooser.addOption("Herra 9 F Curve", JoystickCurves.CurveType.HERRA9_F_CURVE);
        linearCurveChooser.addOption("Herra Mixed", JoystickCurves.CurveType.HERRA_MIXED);
        linearCurveChooser.addOption("Cheesy Curve", JoystickCurves.CurveType.CHEESY_CURVE);
        SmartDashboard.putData("linear curve chooser", linearCurveChooser);

        angularCurveChooser.setDefaultOption("Linear", JoystickCurves.CurveType.LINEAR);
        angularCurveChooser.addOption("Power", JoystickCurves.CurveType.POWER);
        angularCurveChooser.addOption("Exponential", JoystickCurves.CurveType.EXPONENTIAL);
        angularCurveChooser.addOption("FS2 Default", JoystickCurves.CurveType.FS2_DEFAULT);
        angularCurveChooser.addOption("FS2 Windows", JoystickCurves.CurveType.FS2_WINDOWS);
        angularCurveChooser.addOption("Herra 4.5 F Curve", JoystickCurves.CurveType.HERRA4_5_F_CURVE);
        angularCurveChooser.addOption("Herra 9 F Curve", JoystickCurves.CurveType.HERRA9_F_CURVE);
        angularCurveChooser.addOption("Herra Mixed", JoystickCurves.CurveType.HERRA_MIXED);
        angularCurveChooser.addOption("Cheesy Curve", JoystickCurves.CurveType.CHEESY_CURVE);
        SmartDashboard.putData("angular curve chooser", angularCurveChooser);

        //we want to be able to measure max speed and acceleration for auto and what not
        SmartDashboard.putNumber("max linear speed", maxLinearSpeed);
        SmartDashboard.putNumber("max linear acceleration", maxLinearAcceleration);
        SmartDashboard.putNumber("max angular velocity", 0);
        SmartDashboard.putNumber("max angular acceleration", 0);
        SmartDashboard.putNumber("linear speed", 0);
        SmartDashboard.putNumber("linear acceleration", 0);
        SmartDashboard.putNumber("angular velocity", 0);
        SmartDashboard.putNumber("angular acceleration", 0);

        SmartDashboard.putBoolean("reset max measurements", false);

        System.err.println("WARNING: You have instantiated DriveTrainTuneable. Unless you are Sean, or have asked Sean, or are Sean's clone, you should not be doing this. (unless you're on another team. in which case, hi!)");
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

        swerve.applyDriveConfigs(newDriveGains);
        swerve.applySteerConfigs(newAngleGains);
    }

    @Override
    public void periodic() {
        //do we want to reset the drivetrain with new constants?
        if(SmartDashboard.getBoolean("update drive", false)) {
            updateDrive();
            SmartDashboard.putBoolean("update drive", false);
        }

        //let me graph the errors for pid tuning
        SmartDashboard.putNumberArray("drive motor errors", swerve.getDriveClosedLoopErrors());
        SmartDashboard.putNumberArray("angle motor errors", swerve.getAngleClosedLoopErrors());

        //only drive if we want to
        if(!SmartDashboard.getBoolean("enable tuning drive", false)) return;

        double xVelocity = driverController.getRawAxis(5);
        double yVelocity = driverController.getRawAxis(4);
        double angularVelocity = driverController.getRawAxis(0);

        double autoHeadingAngle = SmartDashboard.getNumber("auto heading angle", 0);
        boolean isAutoHeading = SmartDashboard.getBoolean("auto heading", false);
        boolean isFieldRelative = SmartDashboard.getBoolean("field relative", true);
        boolean isOpenLoop = SmartDashboard.getBoolean("open loop", true);
        boolean isLockIn = SmartDashboard.getBoolean("lock in", false);


        if(SmartDashboard.getBoolean("zero odometry", false)) {
            swerve.tareEverything();
            SmartDashboard.putBoolean("zero odometry", false);
        }

        if(SmartDashboard.getBoolean("reset max measurements", false)) {
            maxLinearSpeed = 0;
            maxLinearAcceleration = 0;
            maxAngularVelocity = 0;
            maxAngularAcceleration = 0;
            SmartDashboard.putBoolean("reset max measurements", false);
        }

        //handle max linear speed and acceleration
        var currentChassisSpeed = swerve.getChassisSpeeds();
        Translation2d currentLinearVelocity = new Translation2d(currentChassisSpeed.vxMetersPerSecond, currentChassisSpeed.vyMetersPerSecond);
        double currentLinearSpeed = currentLinearVelocity.getNorm();
        double currentLinearAcceleration = (currentLinearSpeed - lastChassisSpeed.getNorm())/Constants.period;
        lastChassisSpeed = currentLinearVelocity;

        maxLinearSpeed = Math.max(maxLinearSpeed, currentLinearSpeed);
        maxLinearAcceleration = Math.max(maxLinearAcceleration, currentLinearAcceleration);

        SmartDashboard.putNumber("max linear speed", maxLinearSpeed);
        SmartDashboard.putNumber("max linear acceleration", maxLinearAcceleration);
        SmartDashboard.putNumber("linear speed", currentLinearSpeed);
        SmartDashboard.putNumber("linear acceleration", currentLinearAcceleration);

        //handle max angular velocity and acceleration
        double currentAngularVelocity = currentChassisSpeed.omegaRadiansPerSecond;
        double currentAngularAcceleration = (currentAngularVelocity - lastRotationalSpeed)/0.02;
        lastRotationalSpeed = currentAngularVelocity;

        maxAngularVelocity = Math.max(maxAngularVelocity, currentAngularVelocity);
        maxAngularAcceleration = Math.max(maxAngularAcceleration, currentAngularAcceleration);

        SmartDashboard.putNumber("max angular velocity", maxAngularVelocity);
        SmartDashboard.putNumber("max angular acceleration", maxAngularAcceleration);
        SmartDashboard.putNumber("angular velocity", currentAngularVelocity);
        SmartDashboard.putNumber("angular acceleration", currentAngularAcceleration);
    


        angularCurveSensitivity = SmartDashboard.getNumber("angular curve sensitivity", angularCurveSensitivity);
        angularVelocity = JoystickCurves.curve(angularCurveChooser.getSelected(), angularCurveSensitivity, angularVelocity);

        // handle smoothing and deadbanding
        Translation2d linearVelocity = new Translation2d(xVelocity, yVelocity);
        linearVelocity = smoothAndDeadband(linearVelocity).times(newTeleopLinearMultiplier);
        SmartDashboard.putNumber("requested linear velocity", linearVelocity.getNorm());
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

    @Override
    public void simulationPeriodic() {
        swerve.updateSimState(Constants.period,12);
    }

    
    private Translation2d smoothAndDeadband (Translation2d linearVelocity) {
        //handle deadband and reset the rate limiter if we're in the deadband
        double rawLinearSpeed = handleDeadbandFixSlope(linearSpeedDeadband,linearVelocity.getNorm());
        if(Math.abs(rawLinearSpeed) < linearSpeedDeadband) linearSpeedLimiter.reset(0);

        linearCurveSensitivity = SmartDashboard.getNumber("linear curve sensitivity", linearCurveSensitivity);

        rawLinearSpeed = JoystickCurves.curve(linearCurveChooser.getSelected(), linearCurveSensitivity, rawLinearSpeed);

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


