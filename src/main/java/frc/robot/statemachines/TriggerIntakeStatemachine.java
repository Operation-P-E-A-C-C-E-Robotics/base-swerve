package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;

public class TriggerIntakeStatemachine extends Command {
    private TriggerIntakeState state = TriggerIntakeState.RETRACT;

    /**
     * Update the desired state of the mechanism
     * This is where the logic for the state machine goes.
     * (e.g. transitioning from intaking to resting when the game piece is detected)
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
     * @param state the desired state
     */
    public void requestState(TriggerIntakeState state){

    }

    /**
     * Make the mechanism attain the desired state
     */
    @Override
    public void execute(){
        updateState();
        switch(state) {
            default:
                break;
            
        }
    }

    public TriggerIntakeState getState(){
        return state;
    }

    enum TriggerIntakeState{
        //TODO
        RETRACT(0.0,0.0),
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

        private TriggerIntakeState (Double deployAngle, Double speed){
            this.deployAngle = deployAngle;
            this.speed = speed;
        }
    }
}
