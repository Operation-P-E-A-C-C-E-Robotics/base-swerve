package frc.robot.statemachines;

import frc.lib.state.StateMachine;

public class FlywheelIntakeStatemachine extends StateMachine<FlywheelIntakeStatemachine.FlywheelIntakeState>{
    private FlywheelIntakeState state = FlywheelIntakeState.RETRACT;

    private void updateState(){
        switch (state) {
            default:
                break;
        }
    }

    @Override
    public void requestState(FlywheelIntakeState state){

    }

    @Override
    public void update(){
        updateState();
        switch(state) {
            default:
                break;
            
        }
    }

    @Override
    public FlywheelIntakeState getState(){
        return state;
    }

    @Override
    public boolean isDone(){
        switch(state){
            default:
                return true;
        }
    }

    public enum FlywheelIntakeState{
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

        private FlywheelIntakeState (Double deployAngle, Double speed){
            this.deployAngle = deployAngle;
            this.speed = speed;
        }
    }
}
