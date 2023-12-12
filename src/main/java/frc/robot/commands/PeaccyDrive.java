package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.PeaccyRequest;
import frc.lib.telemetry.SwerveTelemetry;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class PeaccyDrive extends Command {
    private final DoubleSupplier xVelocitySup, yVelocitySup, angularVelocitySup, autoHeadingAngleSup;
    private final BooleanSupplier isAutoAngleSup, isFieldRelativeSup, isOpenLoopSup, isLockInSup;
    private final DriveTrain driveTrain;

    private final PeaccyRequest request;
    private final SwerveRequest.SwerveDriveBrake lockInRequest = new SwerveRequest.SwerveDriveBrake().withIsOpenLoop(false);

    private final SlewRateLimiter linearSpeedLimiter = new SlewRateLimiter(Constants.Swerve.teleopLinearSpeedLimit);
    private final SlewRateLimiter lowBatteryLinearSpeedLimiter = new SlewRateLimiter(Constants.Swerve.teleopLowBatteryLinearSpeedLimit);
    private final SlewRateLimiter linearAngleLimiter = new SlewRateLimiter(Constants.Swerve.teleopLinearAngleLimit);
    private final SlewRateLimiter nearLinearAngleLimiter = new SlewRateLimiter(Constants.Swerve.teleopNearLinearAngleLimit); //more extreme limit near zero 
    private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(Constants.Swerve.teleopAngularRateLimit);
    private BooleanSupplier isZeroOdometrySup;

    private final Debouncer linearDeadbandDebouncer = new Debouncer(0.15, DebounceType.kBoth);
    private final Debouncer angularDeadbandDebouncer = new Debouncer(0.15, DebounceType.kBoth);

    private final Timer robotNotMovingTimer = new Timer();
    private final Timer robotMovingTimer = new Timer();


    /**
     * PeaccyDrive is a swerve drive command designed to handle all the different
     * modes of driving that we want to use.
     *
     * It has advanced input smoothing and deadbanding,
     * field centric and robot centric modes,
     * and auto angle (automatic heading adjustment) modes.
     * @param xVelocitySup the requested x velocity
     * @param yVelocitySup the requested y velocity
     * @param angularVelocitySup the requested angular velocity
     * @param autoHeadingAngleSup the requested auto heading angle
     * @param isAutoAngleSup whether or not to use automatic heading adjustment
     * @param isFieldRelativeSup whether or not to use field centric mode
     * @param isOpenLoopSup whether or not to use open loop controls
     * @param isLockInSup whether or not to X-lock the wheels when stopped
     * @param isZeroOdometry whether or not to zero the odometry
     * @param driveTrain the swerve subsystem
     */
    public PeaccyDrive(DoubleSupplier xVelocitySup,
                      DoubleSupplier yVelocitySup,
                      DoubleSupplier angularVelocitySup,
                      DoubleSupplier autoHeadingAngleSup,
                      BooleanSupplier isAutoAngleSup,
                      BooleanSupplier isFieldRelativeSup,
                      BooleanSupplier isOpenLoopSup,
                      BooleanSupplier isLockInSup,
                      BooleanSupplier isZeroOdometry,
                      DriveTrain driveTrain) {
        this.xVelocitySup = xVelocitySup;
        this.yVelocitySup = yVelocitySup;
        this.angularVelocitySup = angularVelocitySup;
        this.autoHeadingAngleSup = autoHeadingAngleSup;
        this.isAutoAngleSup = isAutoAngleSup;
        this.isFieldRelativeSup = isFieldRelativeSup;
        this.isOpenLoopSup = isOpenLoopSup;
        this.isLockInSup = isLockInSup;
        this.isZeroOdometrySup = isZeroOdometry;
        this.driveTrain = driveTrain;

        request  = new PeaccyRequest(
            50, 
            70,
            2600, 
            0.0, 
            0.0, 
            driveTrain::getChassisSpeeds, 
            driveTrain::getTotalDriveCurrent, 
            30
        ).withRotationalDeadband(Constants.Swerve.teleopAngularVelocityDeadband)
        .withSoftHoldHeading(false)
        .withPositionCorrectionIterations(4)
        .withPositionCorrectionWeight(1);

        robotMovingTimer.start();
        robotNotMovingTimer.start();

        addRequirements(driveTrain);
    }

    @Override
    public void initialize(){
        driveTrain.resetOdometry();
        request.withHeading(driveTrain.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        double xVelocity = xVelocitySup.getAsDouble();
        double yVelocity = yVelocitySup.getAsDouble();
        double angularVelocity = angularVelocitySup.getAsDouble();
        double autoHeadingAngle = autoHeadingAngleSup.getAsDouble();

        boolean isAutoHeading = isAutoAngleSup.getAsBoolean();
        boolean isFieldRelative = isFieldRelativeSup.getAsBoolean();
        boolean isOpenLoop = isOpenLoopSup.getAsBoolean();
        boolean isLockIn = isLockInSup.getAsBoolean();
        boolean isZeroOdometry = isZeroOdometrySup.getAsBoolean();

        if(isZeroOdometry) {
            driveTrain.resetOdometry();
            request.withHeading(driveTrain.getPose().getRotation().getRadians());
        }

        // handle smoothing and deadbanding
        Translation2d linearVelocity = new Translation2d(xVelocity, yVelocity);
        linearVelocity = smoothAndDeadband(linearVelocity).times(Constants.Swerve.teleopLinearMultiplier);
        angularVelocity = smoothAndDeadband(angularVelocity) * Constants.Swerve.teleopAngularMultiplier;

        if(linearVelocity.getNorm() > 0.1) robotNotMovingTimer.reset();
        else robotMovingTimer.reset();

        // log data
        SwerveTelemetry.updateSwerveCommand(
            linearVelocity.getX(), 
            linearVelocity.getY(), 
            angularVelocity, 
            autoHeadingAngle, 
            isAutoHeading, 
            isFieldRelative, 
            isOpenLoop, 
            isLockIn, 
            isZeroOdometry
        );



        //handle lock in
        if (isLockIn) {
            if (linearVelocity.equals(new Translation2d(0,0)) && angularVelocity == 0) {
                driveTrain.drive(lockInRequest.withIsOpenLoop(isOpenLoop));
                return;
            }
        }

       request.withVelocityX(linearVelocity.getX())
            .withVelocityY(linearVelocity.getY())
            .withRotationalRate(angularVelocity)
            .withIsOpenLoop(true)
            .withIsFieldCentric(isFieldRelative)
            .withHoldHeading(true);

        if(isAutoHeading) {
            request.withHeading(Rotation2d.fromDegrees(autoHeadingAngle).getRadians());
        }

        driveTrain.drive(request);
    }

    private Translation2d smoothAndDeadband (Translation2d linearVelocity) {
        //handle deadband and reset the rate limiter if we're in the deadband
        double rawLinearSpeed = handleDeadbandFixSlope(Constants.Swerve.teleopLinearSpeedDeadband,0.1,linearVelocity.getNorm(), linearDeadbandDebouncer);
        if(Math.abs(rawLinearSpeed) < Constants.Swerve.teleopLinearSpeedDeadband) linearSpeedLimiter.reset(0);
        rawLinearSpeed = Constants.Swerve.teleopLinearSpeedCurve.apply(rawLinearSpeed);

        boolean useLowBetteryLimiter = RobotController.getBatteryVoltage() < 10.5;

        double lowBatSpeed = lowBatteryLinearSpeedLimiter.calculate(rawLinearSpeed);
        //limit the linear acceleration
        double linearSpeed = linearSpeedLimiter.calculate(rawLinearSpeed);
        if(useLowBetteryLimiter) linearSpeed = lowBatSpeed;

        boolean useNearLimiter = Math.abs(rawLinearSpeed) < Constants.Swerve.teleopNearLimitThreshold;

        //limit the change in direction
        double rawLinearAngle = linearVelocity.getAngle().getRadians();
        double nearLinearAngle = nearLinearAngleLimiter.calculate(rawLinearAngle);
        double linearAngle = linearAngleLimiter.calculate(rawLinearAngle);
        if(useNearLimiter) linearAngle = nearLinearAngle;

        // override the smoothing of the direction if it lags too far behind the raw value
        // (mainly after stopping and changing direction)
        if (Math.abs(linearAngle - rawLinearAngle) > Math.PI/4) {
            linearAngleLimiter.reset(rawLinearAngle);
            linearAngle = rawLinearAngle;
        }

        return new Translation2d(linearSpeed, new Rotation2d(linearAngle));
    }

    private double smoothAndDeadband (double angularVelocity) {
        //apply deadband to angular velocity
        angularVelocity = handleDeadbandFixSlope(Constants.Swerve.teleopAngularVelocityDeadband, 0.13, angularVelocity, angularDeadbandDebouncer);

        angularVelocity = Constants.Swerve.teleopAngularVelocityCurve.apply(angularVelocity);

        //limit the angular acceleration
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
    private double handleDeadbandFixSlope (double modband, double deadband, double value, Debouncer debounce) {
        if (debounce.calculate(Math.abs(value) < deadband)) return 0;
        var mod = (value - (deadband * Math.signum(value)))/(1 - deadband);
        return Math.abs(mod) > deadband ? mod : 0;
    }
}
