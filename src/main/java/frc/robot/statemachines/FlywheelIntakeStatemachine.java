package frc.robot.statemachines;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.planners.IntakeMotionPlanner;
import frc.robot.subsystems.FlywheelIntake;

public class FlywheelIntakeStatemachine extends StateMachine<FlywheelIntakeStatemachine.FlywheelIntakeState>{
    private FlywheelIntakeState state = FlywheelIntakeState.RETRACT;

    private final FlywheelIntake flywheelIntake;
    private final IntakeMotionPlanner intakeMotionPlanner;
    private final BooleanSupplier hasNote;

    public FlywheelIntakeStatemachine(FlywheelIntake flywheelIntake, IntakeMotionPlanner intakeMotionPlanner, BooleanSupplier hasNote){
        this.flywheelIntake = flywheelIntake;
        this.intakeMotionPlanner = intakeMotionPlanner;
        this.hasNote = hasNote;
    }

    private void updateState(){
        //don't intake until the shooter is ready (otherwise we'd spit it onto the bellypan which would be incredibly bad)
        if(state == FlywheelIntakeState.INTAKE && !intakeMotionPlanner.readyToIntake()) state = FlywheelIntakeState.EXTEND;

        //don't allow the shooter to hit the intake
        if(state == FlywheelIntakeState.RETRACT && intakeMotionPlanner.shouldFlywheelIntakeAvoid()) state = FlywheelIntakeState.AVOID;
        if(state == FlywheelIntakeState.AVOID && !intakeMotionPlanner.shouldFlywheelIntakeAvoid()) state = FlywheelIntakeState.RETRACT;

        //automatically transition to retracting the intake once we have detected a note
        if(state == FlywheelIntakeState.INTAKE && hasNote.getAsBoolean()) state = FlywheelIntakeState.RETRACT; //TODO make sure this isn't prone to sensor failure / noise
    }

    @Override
    public void requestState(FlywheelIntakeState state){
        this.state = state;
    }

    @Override
    public void update(){
        updateState();
        flywheelIntake.setDeploymentAngle(state.deployAngle);
        flywheelIntake.setRollerSpeed(state.speed);

        SmartDashboard.putString("Flywheel Intake State", state.name());
    }

    @Override
    public FlywheelIntakeState getState(){
        return state;
    }

    @Override
    public boolean isDone(){
        return flywheelIntake.deployedToSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return true;
    }

    public enum FlywheelIntakeState{
        //TODO
        RETRACT(0.0,0.0),
        EXTEND(0.0,0.0),
        INTAKE(0.0,0.0),
        AVOID(0.0,0.0),
        EJECT(0.0,0.0);
        
        private Double deployAngle;
        private Double speed;

        public Double getDeployAngle(){
            return deployAngle;
        }

        public Double getSpeed(){
            return speed;
        }

        private FlywheelIntakeState (Double deployAngle, Double speed){
            this.deployAngle = deployAngle;
            this.speed = speed;
        }
    }
}
