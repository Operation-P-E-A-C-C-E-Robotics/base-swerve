package frc.robot.planners;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.LinearInterpolate;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/**
 * In charge of calculating the correct angles for the pivot and drivetrain,
 * and the flywheel velocity, to shoot at.
 */
public class AimPlanner {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotRelativeChassisSpeeds;
    private final BooleanSupplier shootWhileMoving; // enable correction for drivetrain velocity

    private final Translation2d targetCenterTranslation = new Translation2d(
        FieldConstants.Speaker.centerSpeakerOpening.getX(),
        FieldConstants.Speaker.centerSpeakerOpening.getY()
    );

    private final double[][] distanceCalibrationData = {
        {52, 35, 27, 24}, // pivot angles (deg)
        {40, 50, 50, 50}, // flywheel speed rps
        {1, 2, 2.5, 3}  //distances (m)
    };

    private final LinearInterpolate pivotInterpolator = new LinearInterpolate(distanceCalibrationData[2], distanceCalibrationData[0]);
    private final LinearInterpolate flywheelAngularVelocityInterpolater = new LinearInterpolate(distanceCalibrationData[2], distanceCalibrationData[1]);

    private Rotation2d drivetrainAngle = new Rotation2d();
    private Rotation2d pivotAngle = new Rotation2d();
    private double flywheelAngularVelocity = 0;

    //what the desired velocity of the different mechanisms are based off the robot's speed, 
    //so we can add a feedforward to them that will match the robots motion to track the target better
    private double drivetrainAngularVelocity = 0;
    private double pivotAngularVelocity = 0;
    private double shooterAngularAcceleration = 0;

    /* TELEMETRY */ //note: SOTM = Shoot on the Move
    private final NetworkTable aimTable = NetworkTableInstance.getDefault().getTable("Aim Planner");
    private final DoublePublisher distanceToTargetPublisher = aimTable.getDoubleTopic("Distance to Target").publish();
    private final DoublePublisher angleToTargetPublisher = aimTable.getDoubleTopic("Angle to Target").publish();
    private final DoublePublisher pivotAnglePublisher = aimTable.getDoubleTopic("Pivot Angle Regression (deg)").publish();
    private final DoublePublisher shooterVelocityPublisher = aimTable.getDoubleTopic("Shooter Velocity Regression (rps)").publish();
    private final DoublePublisher exitVelocityPublisher = aimTable.getDoubleTopic("Exit Velocity Conversion (m/s)").publish();

    private final DoublePublisher sotmPivotAnglePublisher = aimTable.getDoubleTopic("SOTM Corrected Pivot Angle").publish();
    private final DoublePublisher sotmExitVelocityPublisher = aimTable.getDoubleTopic("SOTM Corrected Exit Velocity").publish();
    private final DoublePublisher sotmDrivetrainAnglePublisher = aimTable.getDoubleTopic("SOTM Corrected Drivetrain Angle").publish();

    private final DoublePublisher drivetrainAngularVelocityPublisher = aimTable.getDoubleTopic("SuperSOTM Drivetrain Angular Velocity").publish();
    private final DoublePublisher pivotAngularVelocityPublisher = aimTable.getDoubleTopic("SuperSOTM Pivot Angular Velocity").publish();
    private final DoublePublisher shooterAngularAccelerationPublisher = aimTable.getDoubleTopic("SuperSOTM Shooter Angular Acceleration").publish();

    public AimPlanner (Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotRelativeChassisSpeeds, BooleanSupplier shootWhileMoving) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotRelativeChassisSpeeds = robotRelativeChassisSpeeds;
        this.shootWhileMoving = shootWhileMoving;
    }

    public void update() {
        Pose2d blueOriginPose = robotPoseSupplier.get();
        var blueTargetTranslation = AllianceFlipUtil.apply(targetCenterTranslation);
        double distanceToTarget = blueOriginPose.getTranslation().getDistance(blueTargetTranslation);

        Rotation2d angleToTarget = blueOriginPose.getTranslation().minus(blueTargetTranslation).getAngle();
        Rotation2d pivotAngle = Rotation2d.fromDegrees(pivotInterpolator.interpolate(distanceToTarget));
        double flywheelAngularVelocity = flywheelAngularVelocityInterpolater.interpolate(distanceToTarget);
        double exitVelocity = RPSToExitVelocity(flywheelAngularVelocity);

        distanceToTargetPublisher.accept(distanceToTarget);
        angleToTargetPublisher.accept(angleToTarget.getDegrees());
        pivotAnglePublisher.accept(pivotAngle.getDegrees());
        shooterVelocityPublisher.accept(flywheelAngularVelocity);
        exitVelocityPublisher.accept(exitVelocity);

        if(!shootWhileMoving.getAsBoolean()) {
            drivetrainAngularVelocity = 0;
            pivotAngularVelocity = 0;
            shooterAngularAcceleration = 0;
            this.pivotAngle = pivotAngle;
            this.flywheelAngularVelocity = flywheelAngularVelocity;
            this.drivetrainAngle = angleToTarget;
            return;
        }

        ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeChassisSpeeds.get(), blueOriginPose.getRotation());
        ShotAngle uncorrectedShotAngle = new ShotAngle(angleToTarget, pivotAngle, exitVelocity);
        ShotAngle correctedShotAngle = ShotAngle.correctFromChassisSpeeds(uncorrectedShotAngle, robotVelocity, blueOriginPose.getRotation());
        this.pivotAngle = correctedShotAngle.getPivotAngle();
        this.flywheelAngularVelocity = exitVelocityToRPS(correctedShotAngle.getExitVelocity());
        this.drivetrainAngle = correctedShotAngle.getDrivetrainAngle();

        sotmPivotAnglePublisher.accept(pivotAngle.getDegrees());
        sotmExitVelocityPublisher.accept(uncorrectedShotAngle.getExitVelocity());
        sotmDrivetrainAnglePublisher.accept(uncorrectedShotAngle.getDrivetrainAngle().getDegrees());

        //calculate the angular velocities of the mechanisms
        //first get the robots velocity relative to the target
        Translation2d robotVelocityTranslation = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
        Translation2d targetRelativeVelocity = robotVelocityTranslation.rotateBy(angleToTarget);
        double deltaDistance = targetRelativeVelocity.getX();

        //use the distarce to the target to get the angular velocity
        drivetrainAngularVelocity = Units.radiansToDegrees((targetRelativeVelocity.getY() / distanceToTarget));
        pivotAngularVelocity = pivotInterpolator.derivative(distanceToTarget) * deltaDistance;
        shooterAngularAcceleration = flywheelAngularVelocityInterpolater.derivative(distanceToTarget) * deltaDistance;

        drivetrainAngularVelocityPublisher.accept(drivetrainAngularVelocity);
        pivotAngularVelocityPublisher.accept(pivotAngularVelocity);
        shooterAngularAccelerationPublisher.accept(shooterAngularAcceleration);
    }

    public Rotation2d getTargetDrivetrainAngle() {
        return drivetrainAngle;
    }

    public Rotation2d getTargetPivotAngle() {
        return pivotAngle;
    }

    public double getTargetFlywheelVelocityRPS() {
        return flywheelAngularVelocity;
    }

    public double getDrivetrainAngularVelocity() {
        return drivetrainAngularVelocity;
    }

    public double getPivotAngularVelocity() {
        return pivotAngularVelocity;
    }

    public double getShooterAngularAcceleration() {
        return shooterAngularAcceleration;
    }

    private double exitVelocityToRPS(double exitVelocity) {
        return (exitVelocity / (Constants.Shooter.flywheelDiameter * Math.PI) / Constants.Shooter.flywheelGearRatio) * Constants.Shooter.flywheelEfficiency;
    }

    private double RPSToExitVelocity(double rps) {
        return (rps * Constants.Shooter.flywheelDiameter * Math.PI * Constants.Shooter.flywheelGearRatio) / Constants.Shooter.flywheelEfficiency;
    }

    public static class ShotAngle {
        private final Rotation2d drivetrainAngle;
        private final Rotation2d pivotAngle;
        private final double exitVelocity;

        public ShotAngle(Rotation2d drivetrainAngle, Rotation2d pivotAngle, double exitVelocity) {
            this.drivetrainAngle = drivetrainAngle;
            this.pivotAngle = pivotAngle;
            this.exitVelocity = exitVelocity;
        }

        public Rotation2d getDrivetrainAngle() {
            return drivetrainAngle;
        }

        public Rotation2d getPivotAngle() {
            return pivotAngle;
        }

        public double getExitVelocity() {
            return exitVelocity;
        }

        public Translation3d getRobotCentricVector () {
            return new Translation3d(exitVelocity, new Rotation3d(0, -pivotAngle.getRadians(), drivetrainAngle.getRadians()));
        }

        public Translation3d getFieldCentricVector (Rotation2d robotAngle) {
            return getRobotCentricVector().rotateBy(new Rotation3d(0, 0, robotAngle.getRadians()));
        }

        public static ShotAngle fromRobotCentricVector (Translation3d vector) {
            return new ShotAngle(
                new Rotation2d(vector.getX(), vector.getY()),
                new Rotation2d(vector.getX(), vector.getZ()),
                vector.getNorm()
            );
        }

        public static ShotAngle fromFieldCentricVector (Translation3d vector, Rotation2d robotAngle) {
            return fromRobotCentricVector(vector.rotateBy(new Rotation3d(0, 0, -robotAngle.getRadians())));
        }

        public static ShotAngle correctFromChassisSpeeds (ShotAngle shotAngle, ChassisSpeeds speeds, Rotation2d heading) {
            var robotVelocity = new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
            var correctedVector = shotAngle.getFieldCentricVector(heading).minus(robotVelocity);
            return fromFieldCentricVector(correctedVector, heading);
        }

        public String toString () {
            return String.format("Drivetrain Angle: %f, Pivot Angle: %f, Exit Velocity: %f", drivetrainAngle.getDegrees(), pivotAngle.getDegrees(), exitVelocity);
        }
    }

    public boolean readyToShoot() {
        // TODO Auto-generated method stub
        return false;
    }
}
