package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 * SeanyDrive is a swerve drive command designed to handle all the different
 * modes of driving that we want to use.
 *
 * It has advanced input smoothing and deadbanding,
 * field centric and robot centric modes,
 * and auto angle (automatic heading adjustment) modes.
 */
public class SeanyDrive extends Command {
    private final DoubleSupplier xVelocitySup, yVelocitySup, angularVelocitySup, autoHeadingAngleSup;
    private final BooleanSupplier isAutoAngleSup, isFieldRelativeSup, isOpenLoopSup, isLockInSup;
    private final DriveTrain driveTrain;

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.FieldCentricFacingAngle autoHeadingRequest = new SwerveRequest.FieldCentricFacingAngle();
    private final SwerveRequest.SwerveDriveBrake lockInRequest = new SwerveRequest.SwerveDriveBrake();

    private final double linearSpeedDeadband = 0.02,
                         angularVelocityDeadband = 0.02;
    private final SlewRateLimiter linearSpeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter linearAngleLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(3);
    private BooleanSupplier isZeroOdometry;


    public SeanyDrive(DoubleSupplier xVelocitySup,
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
        this.isZeroOdometry = isZeroOdometry;
        this.driveTrain = driveTrain;
        autoHeadingRequest.HeadingController.setP(2);
        addRequirements(driveTrain);
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

        System.out.println(autoHeadingAngle);
        System.out.println(isAutoHeading);

        if(isZeroOdometry.getAsBoolean()) driveTrain.resetOdometry();

        Translation2d linearVelocity = new Translation2d(xVelocity, yVelocity);
        linearVelocity = smoothAndDeadband(linearVelocity).times(5);
        angularVelocity = smoothAndDeadband(angularVelocity) * 5;

        SmartDashboard.putNumberArray("swerve requested velocity", new double[] {linearVelocity.getX(), linearVelocity.getY(), angularVelocity});
        SmartDashboard.putBoolean("isAutoHeading", isAutoHeading);
        SmartDashboard.putBoolean("isFieldRelative", isFieldRelative);
        SmartDashboard.putBoolean("isOpenLoop", isOpenLoop);
        SmartDashboard.putBoolean("isLockIn", isLockIn);



        //handle lock in
        if (isLockIn) {
            if (linearVelocity.equals(new Translation2d(0,0)) && angularVelocity == 0) {
                driveTrain.drive(lockInRequest.withIsOpenLoop(isOpenLoop));
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

            driveTrain.drive(autoHeadingRequest);
            return;
        }

        //handle field relative
        if (isFieldRelative) {
            fieldCentricRequest.withIsOpenLoop(isOpenLoop)
                                .withVelocityX(linearVelocity.getX())
                                .withVelocityY(linearVelocity.getY())
                                .withRotationalRate(angularVelocity);

            driveTrain.drive(fieldCentricRequest);
            return;
        }

        //handle robot relative
        robotCentricRequest.withIsOpenLoop(isOpenLoop)
                            .withVelocityX(linearVelocity.getX())
                            .withVelocityY(linearVelocity.getY())
                            .withRotationalRate(angularVelocity);

        driveTrain.drive(robotCentricRequest);
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

    private double handleDeadbandFixSlope (double deadband, double value) {
        if (Math.abs(value) < deadband) return 0;
        return (value - (deadband * Math.signum(value)))/(1 - deadband);
    }
}
