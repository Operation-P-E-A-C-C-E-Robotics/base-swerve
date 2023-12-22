package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.PeaccyRequest;
import frc.lib.telemetry.SwerveTelemetry;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class PeaccyDrive extends Command {
    /* Initialize the suppliers to default values */
    private DoubleSupplier xVelocitySup = () -> 0,
                           yVelocitySup = () -> 0, 
                           angularVelocitySup = () -> 0, 
                           autoHeadingAngleSup = () -> 0;
    private BooleanSupplier isAutoAngleSup = () -> false, 
                            isFieldRelativeSup = () -> false, 
                            isOpenLoopSup = () -> true, 
                            isLockInSup = () -> false,
                            isZeroOdometrySup = () -> false;

    private DriveTrain driveTrain;

    /* "Swerve Requests" are what the drivetrain subsystem accepts. They figure out how to orient and drive the wheels. */
    private final PeaccyRequest request; //custom fancy request than handles everything
    private final SwerveRequest.SwerveDriveBrake lockInRequest = new SwerveRequest.SwerveDriveBrake().withIsOpenLoop(false); //for X-locking the wheels

    /* Acceleration limiters for a consistent feel and to reduce power draw. */
    private final SlewRateLimiter linearSpeedLimiter = new SlewRateLimiter(Constants.Swerve.teleopLinearSpeedLimit);
    private final SlewRateLimiter lowBatteryLinearSpeedLimiter = new SlewRateLimiter(Constants.Swerve.teleopLowBatteryLinearSpeedLimit);
    private final SlewRateLimiter linearAngleLimiter = new SlewRateLimiter(Constants.Swerve.teleopLinearAngleLimit);
    private final SlewRateLimiter nearLinearAngleLimiter = new SlewRateLimiter(Constants.Swerve.teleopNearLinearAngleLimit); //more extreme limit near zero 
    private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(Constants.Swerve.teleopAngularRateLimit);

    /* Debounce the deadband to prevent jitter when the joystick is near the edge of the deadband */
    private final Debouncer linearDeadbandDebouncer = new Debouncer(Constants.Swerve.teleopDeadbandDebounceTime, DebounceType.kBoth);
    private final Debouncer angularDeadbandDebouncer = new Debouncer(Constants.Swerve.teleopDeadbandDebounceTime, DebounceType.kBoth);

    private FallbackMode fallback = FallbackMode.Normal; //disable risky features

    /**
     * PeaccyDrive is a swerve drive command designed to handle all the different
     * modes of driving that we want to use.
     *
     * It has advanced input smoothing and deadbanding,
     * field centric and robot centric modes,
     * and auto angle (automatic heading adjustment) modes.
     * @param driveTrain the swerve subsystem
     */
    public PeaccyDrive(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;

        request  = new PeaccyRequest(
            Constants.Swerve.autoHeadingMaxVelocity, 
            Constants.Swerve.autoHeadingMaxAcceleration,
            Constants.Swerve.autoHeadingKP, 
            Constants.Swerve.autoHeadingKV, 
            Constants.Swerve.autoHeadingKA, 
            driveTrain::getChassisSpeeds, 
            driveTrain::getTotalDriveCurrent, 
            Constants.Swerve.softHeadingCurrentLimit
        ).withRotationalDeadband(Constants.Swerve.teleopAngularVelocityDeadband)
        .withSoftHoldHeading(Constants.Swerve.useSoftHoldHeading)
        .withPositionCorrectionIterations(Constants.Swerve.teleopPositionCorrectionIters);

        addRequirements(driveTrain);
        System.out.println("PeacyDrive initialized");
    }

    /**
     * give the supplier to control robot translatoin
     * @param xVelocitySup supplier that gives desired x velocity as a percentage of max velocity
     * @return this (so u can chain em)
     */
    public PeaccyDrive withTranslation(DoubleSupplier xVelocitySup){
        this.xVelocitySup = xVelocitySup;
        return this;
    }

    /**
     * give the supplier to control robot strafe
     * @param yVelocitySup supplier that gives desired y velocity as a percentage of max velocity
     * @return this (so u can chain em)
     */
    public PeaccyDrive withStrafe(DoubleSupplier yVelocitySup){
        this.yVelocitySup = yVelocitySup;
        return this;
    }

    /**
     * give the supplier to control robot rotation
     * @param angularVelocitySup supplier that gives desired angular velocity as a percentage of max velocity
     * @return this (so u can chain em)
     */
    public PeaccyDrive withRotation(DoubleSupplier angularVelocitySup){
        this.angularVelocitySup = angularVelocitySup;
        return this;
    }

    /**
     * give the supplier to control robot heading - will be overrided by rotation, if the rotation value is nonzero (below the deadband)
     * @param headingSup supplier that gives desired heading in field-centric degrees
     * @return this (so u can chain em)
     */
    public PeaccyDrive withHeading(DoubleSupplier headingSup){
        this.autoHeadingAngleSup = headingSup;
        return this;
    }

    /**
     * give the supplier for whether to X-lock the wheels when stopped
     * @param isLockInSup supplier that gives whether to X-lock the wheels when stopped
     * @return this (so u can chain em)
     */
    public PeaccyDrive isLockIn(BooleanSupplier isLockInSup){
        this.isLockInSup = isLockInSup;
        return this;
    }

    /**
     * give the supplier for whether to use field centric controls
     * @param isFieldRelativeSup supplier that gives whether to use field centric controls
     * @return this (so u can chain em)
     */
    public PeaccyDrive isFieldRelative(BooleanSupplier isFieldRelativeSup){
        this.isFieldRelativeSup = isFieldRelativeSup;
        return this;
    }

    /**
     * give the supplier for whether to use open loop controls (no velocity controllers on the drive motors)
     * @param isOpenLoopSup supplier that gives whether to use open loop controls
     * @return this (so u can chain em)
     */
    public PeaccyDrive isOpenLoop(BooleanSupplier isOpenLoopSup){
        this.isOpenLoopSup = isOpenLoopSup;
        return this;
    }

    /**
     * give the supplier for whether to zero the odometry (use with caution ofc since this resets field position)
     * @param isZeroOdometrySup supplier that gives whether to zero the odometry
     * @return this (so u can chain em)
     */
    public PeaccyDrive isZeroOdometry(BooleanSupplier isZeroOdometrySup){
        this.isZeroOdometrySup = isZeroOdometrySup;
        return this;
    }

    /**
     * whether to update the robot's target heading using the specified heading.
     * (otherwise, withHeading will be ignored)
     * @param useHeadingSup supplier that gives whether to use the heading
     * @return this (so u can chain em)
     */
    public PeaccyDrive useHeading(BooleanSupplier useHeadingSup){
        this.isAutoAngleSup = useHeadingSup;
        return this;
    }

    @Override
    public void initialize(){
        // driveTrain.resetOdometry();
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
        Translation2d rawLinearVelocity = new Translation2d(xVelocity, yVelocity);
        Translation2d linearVelocity = smoothAndDeadband(rawLinearVelocity).times(Constants.Swerve.teleopLinearMultiplier);
        angularVelocity = smoothAndDeadband(angularVelocity) * Constants.Swerve.teleopAngularMultiplier;

        rawLinearVelocity = rawLinearVelocity.times(Constants.Swerve.teleopLinearMultiplier); //apples to apples comparison for logging

        // log data
        SwerveTelemetry.updateSwerveCommand(
            linearVelocity.getX(), 
            linearVelocity.getY(), 
            rawLinearVelocity.getX(),
            rawLinearVelocity.getY(),
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
            //only lock in if we're stopped
            if (linearVelocity.equals(new Translation2d(0,0)) && angularVelocity == 0) {
                //use CTRE lock in, then return so it's not overridden by the normal request :)
                driveTrain.drive(lockInRequest.withIsOpenLoop(isOpenLoop));
                return;
            }
        }

       request.withVelocityX(linearVelocity.getX())
            .withVelocityY(linearVelocity.getY())
            .withRotationalRate(angularVelocity)
            .withIsOpenLoop(isOpenLoop)
            .withIsFieldCentric(isFieldRelative)
            .withHoldHeading(true) //always hold our current heading.
            .withPositionCorrectionIterations(4);

        //update the robot's target heading if we're using auto heading
        if(isAutoHeading) {
            request.withHeading(Rotation2d.fromDegrees(autoHeadingAngle).getRadians());
        }

        //disable risky features if we're in fallback mode
        if(fallback != FallbackMode.Normal){
            request.withPositionCorrectionIterations(0); //position correction depends on odometry
        }
        if(fallback == FallbackMode.PigeonFailure){
            //these depend on the robot's angle
            request.withIsFieldCentric(false);
            request.withHoldHeading(false);
            request.withSoftHoldHeading(false);
        }

        //yay peaccyrequest will handle the rest :D
        driveTrain.drive(request);
    }

    private Translation2d smoothAndDeadband (Translation2d linearVelocity) {
        //handle deadband and reset the rate limiter if we're in the deadband
        double rawLinearSpeed = handleDeadbandFixSlope(Constants.Swerve.teleopLinearSpeedDeadband,0.03,linearVelocity.getNorm(), linearDeadbandDebouncer);
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
        angularVelocity = handleDeadbandFixSlope(Constants.Swerve.teleopAngularVelocityDeadband, 0.3, angularVelocity, angularDeadbandDebouncer);

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

    public void fallback(){
        if(fallback == FallbackMode.Normal) fallback = FallbackMode.OdmetryFailure;
        if(fallback == FallbackMode.OdmetryFailure) fallback = FallbackMode.PigeonFailure;
    }

    public void resetFallback(){
        fallback = FallbackMode.Normal;
    }
    
    public enum FallbackMode{
        Normal,
        OdmetryFailure,
        PigeonFailure
    }
}
