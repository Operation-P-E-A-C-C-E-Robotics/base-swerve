package frc.robot.planners;

import java.util.function.DoubleSupplier;

/**
 * In charge of calculating whether the pivot and flywheel can
 * be flattened and retracted, respectively.
 */
public class IntakeMotionPlanner {
    private final DoubleSupplier pivotAngle;
    private final DoubleSupplier flywheelIntakeExtension;

    //TODO
    private final double interferenceLowerPivotAngle = 0.0; //threshold for the flywheel intake to avoid the pivot
    private final double interferenceUpperPivotAngle = 0.0; //threshold for the trigger intake to avoid the pivot
    private final double interferenceIntakeExtension = 0.0; //threshold for the pivot to be allowed to flatten
    private final double allowIntakeLowerPivotAngle = 0.0; //threshold for the flywheel intake to be allowed to intake

    private boolean canFlattenPivot = false;
    private boolean shouldFlywheelIntakeAvoid = false;
    private boolean shouldTriggerIntakeAvoid = false;
    private boolean readyToIntake = false;

    public IntakeMotionPlanner (DoubleSupplier pivotAngle, DoubleSupplier flywheelIntakeExtension) {
        this.pivotAngle = pivotAngle;
        this.flywheelIntakeExtension = flywheelIntakeExtension;
    }

    public void update() {
        canFlattenPivot = flywheelIntakeExtension.getAsDouble() < interferenceIntakeExtension;
        shouldFlywheelIntakeAvoid = pivotAngle.getAsDouble() < interferenceLowerPivotAngle;
        shouldTriggerIntakeAvoid = pivotAngle.getAsDouble() > interferenceUpperPivotAngle;
        readyToIntake = pivotAngle.getAsDouble() < allowIntakeLowerPivotAngle;
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
}
