package frc.robot.statemachines;

import frc.lib.state.Statemachine;

public class ShooterStatemachine implements Statemachine<ShooterStatemachine.ShooterState> {
    private ShooterState state = ShooterState.REST;

    /**
     * Handle the logic for changing states
     * e.g. intaking to indexing when the gamepiece is detected
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
     * e.g. changing state before we have finished INDEXing
     * @param state
     */
    @Override
    public void requestState(ShooterState state){

    }

    /**
     * make the mechanism attain the desired state
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
    public ShooterState getState(){
        return state;
    }

    @Override
    public boolean isDone(){
        switch(state){
            default:
                return true;
        }
    }

    enum ShooterState{
        REST(0.0,0.0),
        INTAKE(0.0,0.0), //NOTE: this should fold flat if the flywheel-side intake is out
        INDEX,
        HANDOFF(0.0,0.0), //to diverter
        //add shooter setpoints
        AIM,
        SHOOT,
        AUTOSHOOT;

        private Double flywheelVelocity, triggerPercentage;

        public Double getFlywheelVelocity(){
            return flywheelVelocity;
        }

        public Double getTriggerPercentage(){
            return triggerPercentage;
        }

        private ShooterState (Double flywheelVelocity, Double triggerPercentage){
            this.flywheelVelocity = flywheelVelocity;
            this.triggerPercentage = triggerPercentage;
        }

        private ShooterState() {
            flywheelVelocity = Double.NaN;
            triggerPercentage = Double.NaN;
        }
    }
}
