package frc.robot.planners;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * In charge of calculating the correct angles for the pivot and drivetrain,
 * and the flywheel velocity, to shoot at.
 */
public class AimPlanner {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final boolean shootWhileMoving;

    public AimPlanner (Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier, boolean shootWhileMoving) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.shootWhileMoving = shootWhileMoving;
    }

    public void update() {
        //TODO
    }

    public double getTargetDrivetrainAngle() {
        //TODO
        return 0;
    }

    public void getTargetPivotAngle() {
        //TODO
    }

    public void getTargetFlywheelVelocity() {
        //TODO
    }
}
