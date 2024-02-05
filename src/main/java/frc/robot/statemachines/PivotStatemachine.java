package frc.robot.statemachines;

import frc.lib.state.StateMachine;
import frc.robot.Robot;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.CollisionAvoidancePlanner;
import frc.robot.subsystems.Pivot;

public class PivotStatemachine extends StateMachine<PivotStatemachine.PivotState> {
    private PivotState state = PivotState.REST;

    private final Pivot pivot;
    private final AimPlanner aimPlanner;
    private final CollisionAvoidancePlanner intakeMotionPlanner;

    public PivotStatemachine(Pivot pivot, AimPlanner aimPlanner, CollisionAvoidancePlanner intakeMotionPlanner){
        this.pivot = pivot;
        this.aimPlanner = aimPlanner;
        this.intakeMotionPlanner = intakeMotionPlanner;
    }

    /**
     * Handle automatic state transitions
     * e.g. AIM to REST when a gamepiece is fired
     */
    private void updateState(){
    }

    /**
     * Request a state for the mechanism to attain
     * Won't allow for a state that would result in a collision or other dangerous situation
     * e.g. flattening to intake before the intake is extended
     * @param state
     */
    @Override
    public void requestState(PivotState state){
        this.state = state;
    }

    /**
     * Make the mechanism attain the desired state
     */
    @Override
    public void update(){
        updateState();

        if(Robot.isSimulation()) {
            pivot.simulationPeriodic();
        }
    }

    @Override
    public PivotState getState(){
        return state;
    }
    
    @Override
    public boolean isDone() {
       return pivot.atSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return true;
    }

    public enum PivotState {
        REST(0.0),
        INTAKE(1.0),
        AMP(0.0),
        PRE_CLIMB(0.0),
        CLIMB(0.0),
        AIM_LAYUP(0.0),
        AIM_PROTECTED(0.0),
        AUTO_AIM(0.0);

        private Double angle;

        public Double getAngle(){
            return angle;
        }

        private PivotState (Double angle){
            this.angle = angle;
        }
    }
}
