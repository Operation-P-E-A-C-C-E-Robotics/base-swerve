package frc.lib.swerve;

import java.util.LinkedList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.motion.Trajectory;

public class PeaccyRequest implements SwerveRequest {

    //Input parameters
    public double VelocityX = 0;
    public double VelocityY = 0;
    public double RotationalRate = 0;
    public double Heading = 0; //will hold both the target heading when explicitly set, and the current heading when holding the heading
    public double RotationalDeadband = 0;

    public boolean HoldHeading = false; //keeps the robot facing Heading unless RotationalRate is over the RotationalDeadband
    public boolean SoftHoldHeading = false; //scales HoldHeading by an allowed total drive current draw to prevent tread wear and brownouts
    public boolean IsOpenLoop = false; //if true, the robot will not use the drive velocity controller
    public boolean IsFieldCentric = false; //if false, position correction will not be applied

    //integrates the requested velocities on top of the past robot pose to figure out where we actually wanted the robot to be,
    //and modify the requested velocities to compensate for the error
    public double PositionCorrectionIterations = 0; //how many past inputs to integrate
    public double PositionCorrectionWeight = 1;

    //heading controller stuff
    private Trajectory headingTrajectory = new Trajectory(new TrapezoidProfile.State(0, 0)); //make it smooth
    private SimpleMotorFeedforward headingFeedforward = new SimpleMotorFeedforward(0, 0, 0); //make it good
    private double holdHeadingkP = 0; //make it work
    private double holdHeadingVelocity = 0;
    private double holdHeadingAcceleration = 0;
    private Timer holdHeadingTrajectoryTimer = new Timer();
    private DoubleSupplier totalDriveCurrent;
    private double totalDriveCurrentLimit;

    
    //position correction stuff
    private LinkedList<Translation2d> positionCorrectionRealPositions = new LinkedList<>();
    private LinkedList<Translation2d> positionCorrectionRequestedVelocities = new LinkedList<>();

    private Supplier<ChassisSpeeds> getChassisSpeeds;


    /**
     * The most epic swerve request ever. Does all the things.
     * Made by the one and only Peaccy.
     */
    public PeaccyRequest(double maxAngularVelocity, 
                    double maxAngularAcceleration, 
                    double holdHeadingkP, 
                    double holdHeadingkV, 
                    double holdHeadingkA, 
                    Supplier<ChassisSpeeds> chassisSpeedsSupplier,
                    DoubleSupplier totalDriveCurrentSupplier,
                    double softHeadingCurrentLimit) {
        holdHeadingAcceleration = maxAngularAcceleration;
        holdHeadingVelocity = maxAngularVelocity;
        headingFeedforward = new SimpleMotorFeedforward(0, holdHeadingkV, holdHeadingkA);
        this.holdHeadingkP = holdHeadingkP;
        
        this.getChassisSpeeds = chassisSpeedsSupplier;
        this.totalDriveCurrent = totalDriveCurrentSupplier;
        this.totalDriveCurrentLimit = softHeadingCurrentLimit;
    }

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        Translation2d toApplyTranslation = new Translation2d(VelocityX, VelocityY);
        double toApplyRotation = RotationalRate;

        if(IsFieldCentric) toApplyTranslation = applyPositionCorrection(toApplyTranslation, parameters.currentPose, parameters.updatePeriod);

        if(toApplyRotation < RotationalDeadband) {
            toApplyRotation = 0;
            if (HoldHeading || SoftHoldHeading) toApplyRotation = applyAutoHeading(parameters);
        } else {
            //unless the heading is overriden, we want to hold the current heading
            Heading = parameters.currentPose.getRotation().getRadians();
        }

        ChassisSpeeds speeds = ChassisSpeeds.discretize(
            IsFieldCentric ? ChassisSpeeds.fromFieldRelativeSpeeds(
                toApplyTranslation.getX(), 
                toApplyTranslation.getY(), 
                toApplyRotation,
                parameters.currentPose.getRotation()
            ) : ChassisSpeeds.fromRobotRelativeSpeeds(
                toApplyTranslation.getX(), 
                toApplyTranslation.getY(), 
                toApplyRotation,
                parameters.currentPose.getRotation()
            ),
            parameters.updatePeriod
        );

        var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], IsOpenLoop);
        }

        
        return StatusCode.OK;
    }

    public PeaccyRequest withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    public PeaccyRequest withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    public PeaccyRequest withRotationalRate(double rotationalRate) {
        this.RotationalRate = rotationalRate;
        return this;
    }

    public PeaccyRequest withHeading(double heading) {
        this.Heading = heading;
        return this;
    }

    public PeaccyRequest withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    public PeaccyRequest withHoldHeading(boolean holdHeading) {
        this.HoldHeading = holdHeading;
        return this;
    }

    public PeaccyRequest withSoftHoldHeading(boolean softHoldHeading) {
        this.SoftHoldHeading = softHoldHeading;
        return this;
    }

    public PeaccyRequest withIsOpenLoop(boolean isOpenLoop) {
        this.IsOpenLoop = isOpenLoop;
        return this;
    }

    public PeaccyRequest withIsFieldCentric(boolean isFieldCentric) {
        this.IsFieldCentric = isFieldCentric;
        return this;
    }

    public PeaccyRequest withPositionCorrectionIterations(double positionCorrectionIterations) {
        this.PositionCorrectionIterations = positionCorrectionIterations;
        return this;
    }

    public PeaccyRequest withPositionCorrectionWeight(double positionCorrectionWeight) {
        this.PositionCorrectionWeight = positionCorrectionWeight;
        return this;
    }

    /**
     * Get the rotation rate to apply to the robot to go to the target heading
     */
    private double applyAutoHeading(SwerveControlRequestParameters parameters) {
        var currentHeading = parameters.currentPose.getRotation().getRadians();

        //make sure our odometry heading is within 180 degrees of the target heading to prevent it from wrapping
        while (Math.abs(currentHeading - Heading) > Math.PI) {
            if (currentHeading > Heading) {
                currentHeading -= 2 * Math.PI;
            } else {
                currentHeading += 2 * Math.PI;
            }
        }

        //regenerate the trajectory if the target heading has changed
        if(Heading != headingTrajectory.getTarget().position) {
            headingTrajectory = Trajectory.trapezoidTrajectory(
                new State(currentHeading, getChassisSpeeds.get().omegaRadiansPerSecond), 
                new State(Heading, 0), 
                holdHeadingVelocity,
                holdHeadingAcceleration 
            );
            holdHeadingTrajectoryTimer.reset();
            holdHeadingTrajectoryTimer.start();
        }

        //calculate the correction
        var target = headingTrajectory.calculate(holdHeadingTrajectoryTimer.get() + parameters.updatePeriod).velocity;
        var feedforward = headingFeedforward.calculate(target);
        var pGain = (target - currentHeading) * holdHeadingkP * parameters.updatePeriod;
        var delta = pGain + feedforward;

        if(SoftHoldHeading){
            //scale by the percentage of allowed current for the correction
            double limitPercentage = totalDriveCurrent.getAsDouble()/totalDriveCurrentLimit;
            delta *= Math.max(1-limitPercentage, 0);
        }

        return delta;
    }
    
    /**
     * Integrate the requested velocities to get the requested position, and compare it to the actual position to get the error.
     * @param requestedTranslation the requested velocity
     * @param currentPose the current pose of the robot
     * @param updatePeriod the time between the last update and this one
     * @return the more gooder velocity
     */
    private Translation2d applyPositionCorrection(Translation2d requestedTranslation, Pose2d currentPose, double updatePeriod){
        if(PositionCorrectionIterations == 0) return requestedTranslation;

        positionCorrectionRealPositions.add(currentPose.getTranslation());
        positionCorrectionRequestedVelocities.add(requestedTranslation);

        if (positionCorrectionRealPositions.size() > PositionCorrectionIterations) {
            positionCorrectionRealPositions.removeFirst();
        }

        if (positionCorrectionRequestedVelocities.size() > PositionCorrectionIterations) {
            positionCorrectionRequestedVelocities.removeFirst();
        }

        //integrate the requested velocities to get the requested position
        Translation2d requestedPositionDelta = new Translation2d(0, 0);
        for (Translation2d requestedVelocity : positionCorrectionRequestedVelocities) {
            requestedPositionDelta = requestedPositionDelta.plus(requestedVelocity.times(updatePeriod));
        }

        Translation2d realPositionDelta = currentPose.getTranslation().minus(positionCorrectionRealPositions.getFirst());
        Translation2d positionError = requestedPositionDelta.minus(realPositionDelta);

        Translation2d newRequestedVelocity = requestedTranslation.plus(positionError.times(PositionCorrectionWeight));
        return newRequestedVelocity;
    }
}
