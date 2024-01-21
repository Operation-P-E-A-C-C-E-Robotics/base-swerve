package frc.robot.planners;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.statemachines.ShooterStatemachine.ShooterState;

/**
 * In charge of calculating whether the pivot and flywheel can
 * be flattened and retracted, respectively.
 */
public class IntakeMotionPlanner {
    private final DoubleSupplier pivotAngle;
    private final DoubleSupplier flywheelIntakeExtension;
    private final Supplier<ShooterState> shooterStateSupplier;
    private final BooleanSupplier shooterReadySupplier;

    public IntakeMotionPlanner (DoubleSupplier pivotAngle, DoubleSupplier flywheelIntakeExtension, Supplier<ShooterState> shooterStateSupplier, BooleanSupplier shooterReadySupplier) {
        this.pivotAngle = pivotAngle;
        this.flywheelIntakeExtension = flywheelIntakeExtension;
        this.shooterStateSupplier = shooterStateSupplier;
        this.shooterReadySupplier = shooterReadySupplier;
    }

    public void update() {
        pivotAngle.getAsDouble();
        //TODO
    }

    public boolean canFlattenPivot() {
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

    public boolean readyToIntake() {
        //TODO
        return false;
    }
}
