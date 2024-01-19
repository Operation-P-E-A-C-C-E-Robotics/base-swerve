package frc.robot.planners;

import java.util.function.DoubleSupplier;

/**
 * In charge of calculating whether the pivot and flywheel can
 * be flattened and retracted, respectively.
 */
public class IntakeMotionPlanner {
    private final DoubleSupplier pivotAngle;
    private final DoubleSupplier flywheelIntakeExtension;

    public IntakeMotionPlanner (DoubleSupplier pivotAngle, DoubleSupplier flywheelIntakeExtension) {
        this.pivotAngle = pivotAngle;
        this.flywheelIntakeExtension = flywheelIntakeExtension;
    }

    public void update() {
        //TODO
    }

    public boolean canFlattenPivot() {
        //TODO
        return false;
    }

    public boolean canRetractFlywheelIntake() {
        //TODO
        return false;
    }

    public boolean shouldFlywheelIntakeAvoid() {
        //TODO
        return false;
    }

    public boolean shouldTriggerIntakeAvoid() {
        //TODO
        return false;
    }
}
