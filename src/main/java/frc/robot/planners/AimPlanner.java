package frc.robot.planners;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.LinearInterpolate;
import frc.robot.Constants;

/**
 * In charge of calculating the correct angles for the pivot and drivetrain,
 * and the flywheel velocity, to shoot at.
 */
public class AimPlanner {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final boolean shootWhileMoving; // enable correction for drivetrain velocity

    private final Translation2d targetCenterTranslation = new Translation2d(0, 0);

    private final double[][] distanceCalibrationData = {
        {90, 80}, // pivot angles (deg)
        {0, 10}, //exit velocities (m/s)
        {0, 1}  //distances (m)
    };

    private final LinearInterpolate pivotInterpolator = new LinearInterpolate(distanceCalibrationData[2], distanceCalibrationData[0]);
    private final LinearInterpolate exitVelocityInterpolator = new LinearInterpolate(distanceCalibrationData[2], distanceCalibrationData[1]);

    private Rotation2d drivetrainAngle = new Rotation2d();
    private Rotation2d pivotAngle = new Rotation2d();
    private double shooterRPS = 0;

    //what the desired velocity of the different mechanisms are based off the robot's speed, 
    //so we can add a feedforward to them that will match the robots motion to track the target better
    private double drivetrainAngularVelocity = 0;
    private double pivotAngularVelocity = 0;
    private double shooterAngularAcceleration = 0;

    public AimPlanner (Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier, boolean shootWhileMoving) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.shootWhileMoving = shootWhileMoving;
    }

    public void update() {
        Pose2d blueOriginPose = AllianceFlipUtil.apply(robotPoseSupplier.get());
        SmartDashboard.putString("aim fixed pose", blueOriginPose.toString());
        double distanceToTarget = blueOriginPose.getTranslation().getDistance(targetCenterTranslation);

        Rotation2d angleToTarget = blueOriginPose.getTranslation().minus(targetCenterTranslation).getAngle();
        Rotation2d pivotAngle = Rotation2d.fromDegrees(pivotInterpolator.interpolate(distanceToTarget));
        double exitVelocity = exitVelocityInterpolator.interpolate(distanceToTarget);

        SmartDashboard.putNumber("Distance to Target", distanceToTarget);
        SmartDashboard.putNumber("Angle to Target", angleToTarget.getDegrees());
        SmartDashboard.putNumber("Pivot Angle", pivotAngle.getDegrees());
        SmartDashboard.putNumber("Exit Velocity", exitVelocity);

        if(!shootWhileMoving) {
            drivetrainAngularVelocity = 0;
            pivotAngularVelocity = 0;
            shooterAngularAcceleration = 0;
            this.pivotAngle = pivotAngle;
            this.shooterRPS = exitVelocityToRPS(exitVelocity);
            this.drivetrainAngle = angleToTarget;
            return;
        }

        ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
        ShotAngle uncorrectedShotAngle = new ShotAngle(angleToTarget, pivotAngle, exitVelocity);
        ShotAngle correctedShotAngle = ShotAngle.correctFromChassisSpeeds(uncorrectedShotAngle, robotVelocity, blueOriginPose.getRotation());
        this.pivotAngle = correctedShotAngle.getPivotAngle();
        this.shooterRPS = exitVelocityToRPS(correctedShotAngle.getExitVelocity());
        this.drivetrainAngle = correctedShotAngle.getDrivetrainAngle();

        SmartDashboard.putNumber("Corrected Pivot Angle", this.pivotAngle.getDegrees());
        SmartDashboard.putNumber("Corrected Exit Velocity", correctedShotAngle.getExitVelocity());
        SmartDashboard.putNumber("Corrected Drivetrain Angle", this.drivetrainAngle.getDegrees());

        //calculate the angular velocities of the mechanisms
        //first get the robots velocity relative to the target
        Translation2d robotVelocityTranslation = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
        Translation2d targetRelativeVelocity = robotVelocityTranslation.rotateBy(angleToTarget);
        double deltaDistance = targetRelativeVelocity.getX();

        //use the distarce to the target to get the angular velocity
        drivetrainAngularVelocity = Units.radiansToDegrees((targetRelativeVelocity.getY() / distanceToTarget));
        pivotAngularVelocity = pivotInterpolator.derivative(distanceToTarget) * deltaDistance;
        shooterAngularAcceleration = exitVelocityInterpolator.derivative(distanceToTarget) * deltaDistance;

        SmartDashboard.putNumber("Drivetrain wanted Angular Velocity", drivetrainAngularVelocity);
        SmartDashboard.putNumber("Pivot Angular Velocity", pivotAngularVelocity);
        SmartDashboard.putNumber("Shooter Angular Acceleration", shooterAngularAcceleration);
    }

    public Rotation2d getTargetDrivetrainAngle() {
        return drivetrainAngle;
    }

    public Rotation2d getTargetPivotAngle() {
        return pivotAngle;
    }

    public double getTargetFlywheelVelocityRPS() {
        return shooterRPS;
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
        return exitVelocity / (Constants.Shooter.flywheelDiameter * Math.PI) / Constants.Shooter.flywheelGearRatio;
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
}
