package frc.lib.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SoftLockHeadingSwerveRequest implements SwerveRequest{
    private PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);
    private DoubleSupplier totalDriveCurrent;
    private double totalDriveCurrentLimit;

    /**
     * A swerve request that is equivalent to @link{FieldCentricSwerveRequest} but when the rotation is under the deadband it
     * uses a gentle position controller to hold the last heading.
     */
    public SoftLockHeadingSwerveRequest(double kP, DoubleSupplier totalDriveCurrent, double totalDriveCurrentLimit) {
        HeadingController.setP(kP);
        this.totalDriveCurrent = totalDriveCurrent;
        this.totalDriveCurrentLimit = totalDriveCurrentLimit;
    }

    /**
     * The velocity in the X direction.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The angular rate to rotate at.
     * Angular rate is defined as counterclockwise positive,
     * so this determines how fast to turn counterclockwise.
     * <p>
     * This is in radians per second
     */
    public double RotationalRate = 0;
    /**
     * The allowable deadband of the request.
     */
    public double Deadband = 0;
    /**
     * Rotational deadband of the request
     */
    public double RotationalDeadband = 0;

    /**
     * True to use open-loop control when driving.
     */
    public boolean IsOpenLoop = true;

    public Rotation2d lastHeading = null;

    /**
     * The last applied state in case we don't have anything to drive
     */
    protected SwerveModuleState[] m_lastAppliedState = null;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        double toApplyX = VelocityX;
        double toApplyY = VelocityY;
        double toApplyOmega = RotationalRate;
        if(lastHeading == null) lastHeading = parameters.currentPose.getRotation();


        if(Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }

        if(Math.abs(toApplyOmega) < RotationalDeadband) {
            double current = parameters.currentPose.getRotation().getRadians();
            double target = lastHeading.getRadians();

            //align the target to be within 180 degrees of the current heading
            while(Math.abs(current - target) >= Math.PI) {
                if(current > target) {
                    target += 2 * Math.PI;
                } else {
                    target -= 2 * Math.PI;
                }
            }

            toApplyOmega = HeadingController.calculate(current, target, parameters.timestamp);

            //make sure our rotation correction isn't using too much current
            double limitPercentage = totalDriveCurrent.getAsDouble()/totalDriveCurrentLimit;
            toApplyOmega *= Math.max(1-limitPercentage, 0);
        } else {
            lastHeading = parameters.currentPose.getRotation();
        }

        ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                    parameters.currentPose.getRotation()), parameters.updatePeriod);

        var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i],  IsOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity);
        }

        return StatusCode.OK;
    }

    public SoftLockHeadingSwerveRequest withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    public SoftLockHeadingSwerveRequest withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    public SoftLockHeadingSwerveRequest withRotationalRate(double rotationalRate) {
        this.RotationalRate = rotationalRate;
        return this;
    }

    public SoftLockHeadingSwerveRequest withDeadband(double deadband) {
        this.Deadband = deadband;
        return this;
    }
    public SoftLockHeadingSwerveRequest withRotationalDeadband(double RotationalDeadband) {
        this.RotationalDeadband = RotationalDeadband;
        return this;
    }

    public SoftLockHeadingSwerveRequest withIsOpenLoop(boolean isOpenLoop) {
        this.IsOpenLoop = isOpenLoop;
        return this;
    }
    
}
