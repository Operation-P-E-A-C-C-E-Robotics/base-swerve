package frc.robot.statemachines;

import frc.lib.state.Statemachine;

public class PivotStatemachine implements Statemachine<PivotStatemachine.PivotState> {
    private PivotState state = PivotState.REST;

    /**
     * Handle automatic state transitions
     * e.g. AIM to REST when a gamepiece is fired
     */
    private void updateState(){
        switch (state) {
            default:
                break;
        }
    }

    /**
     * Request a state for the mechanism to attain
     * Won't allow for a state that would result in a collision or other dangerous situation
     * e.g. flattening to intake before the intake is extended
     * @param state
     */
    @Override
    public void requestState(PivotState state){

    }

    @Override
    public boolean isDone() {
        switch(state) {
            default:
                return true;
        }
    }

    /**
     * Make the mechanism attain the desired state
     */
    @Override
    public void update(){
        updateState();
        switch(state) {
            default:
                break;            
        }
    }

    @Override
    public PivotState getState(){
        return state;
    }

    enum PivotState {
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
