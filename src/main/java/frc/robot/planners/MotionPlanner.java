package frc.robot.planners;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * In charge of calculating whether the pivot and flywheel can
 * be flattened and retracted, respectively.
 * 
 * Also says whether the diverter can extend
 */
public class MotionPlanner {
    private final Supplier<Rotation2d> pivotAngle;
    private final Supplier<Rotation2d> flywheelIntakeExtension;
    private final DoubleSupplier xVelocity;

    //TODO
    //Angles in rotations
    public static final Rotation2d interferenceLowerPivotAngle = Rotation2d.fromDegrees(0); //threshold for the flywheel intake to avoid the pivot
    public static final Rotation2d interferenceUpperPivotAngle = Rotation2d.fromDegrees(0); //threshold for the trigger intake to avoid the pivot
    public static final Rotation2d interferenceIntakeExtension = Rotation2d.fromDegrees(0); //threshold for the pivot to be allowed to flatten
    public static final Rotation2d allowIntakeLowerPivotAngle = Rotation2d.fromDegrees(0); //threshold for the flywheel intake to be allowed to intake
    public static final Rotation2d canDiverterExtendMinPivotAngle = Rotation2d.fromDegrees(0); //threshold for the flipper to be allowed to extend

    private boolean canFlattenPivot = false;
    private boolean shouldFlywheelIntakeAvoid = false;
    private boolean shouldTriggerIntakeAvoid = false;
    private boolean readyToIntake = false;
    private boolean canDiverterExtend = false;

    public MotionPlanner (Supplier<Rotation2d> pivotAngle, Supplier<Rotation2d> flywheelIntakeExtension, DoubleSupplier xVelocity) {
        this.pivotAngle = pivotAngle;
        this.flywheelIntakeExtension = flywheelIntakeExtension;
        this.xVelocity = xVelocity;
    }

    public void update() {
        var pivotRadians = pivotAngle.get().getRadians();
        canFlattenPivot = flywheelIntakeExtension.get().getRadians() < interferenceIntakeExtension.getRadians();
        shouldFlywheelIntakeAvoid = pivotRadians < interferenceLowerPivotAngle.getRadians();
        shouldTriggerIntakeAvoid = pivotRadians > interferenceUpperPivotAngle.getRadians();
        readyToIntake = pivotRadians < allowIntakeLowerPivotAngle.getRadians();
        canDiverterExtend = pivotRadians > canDiverterExtendMinPivotAngle.getRadians();
    }

    public boolean canFlattenPivot() {
        return canFlattenPivot;
    }

    public boolean shouldFlywheelIntakeAvoid() {
        return shouldFlywheelIntakeAvoid;
    }

    public boolean shouldTriggerIntakeAvoid() {
        return shouldTriggerIntakeAvoid;
    }

    public boolean readyToIntake() {
        return readyToIntake;
    }

    public boolean canDiverterExtend() {
        return canDiverterExtend;
    }

    public boolean shouldTransitionToFront() {
        return xVelocity.getAsDouble() > 0.5;
    }

    public boolean shouldTransitionToBack() {
        return xVelocity.getAsDouble() < -0.5;
    }
}
