package frc.robot.statemachines;

import frc.lib.state.StateMachine;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.IntakeMotionPlanner;
import frc.robot.planners.StageAvoidancePlanner;
import frc.robot.subsystems.Pivot;

public class PivotStatemachine extends StateMachine<PivotStatemachine.PivotState> {
    private PivotState state = PivotState.REST;

    private final Pivot pivot;
    private final AimPlanner aimPlanner;
    private final StageAvoidancePlanner stageAvoidancePlanner;
    private final IntakeMotionPlanner intakeMotionPlanner;

    public PivotStatemachine(Pivot pivot, AimPlanner aimPlanner, StageAvoidancePlanner stageAvoidancePlanner, IntakeMotionPlanner intakeMotionPlanner){
        this.pivot = pivot;
        this.aimPlanner = aimPlanner;
        this.stageAvoidancePlanner = stageAvoidancePlanner;
        this.intakeMotionPlanner = intakeMotionPlanner;
    }

    /**
     * Handle automatic state transitions
     * e.g. AIM to REST when a gamepiece is fired
     */
    private void updateState(){
        //don't crash into the stage
        if((state == PivotState.AIM || state == PivotState.AMP) && stageAvoidancePlanner.shouldStow()) state = PivotState.REST;

        //don't crash into the intake
        if(state == PivotState.INTAKE && !intakeMotionPlanner.canFlattenPivot()) state = PivotState.REST;
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
        
        if(state == PivotState.AIM) {
            pivot.setPivotPosition(aimPlanner.getTargetPivotAngle());
            return;
        }
        pivot.setPivotPosition(state.getAngle());
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
        return state == PivotState.AIM || state == PivotState.AMP || state == PivotState.INTAKE;
    }

    public enum PivotState {
        REST(0.0),
        INTAKE(0.0),
        AMP(0.0),
        PRE_CLIMB(0.0),
        CLIMB(0.0),
        //add shooter setpoints
        AIM(0.0);

        private Double angle;

        public Double getAngle(){
            return angle;
        }

        private PivotState (Double angle){
            this.angle = angle;
        }
    }
}
