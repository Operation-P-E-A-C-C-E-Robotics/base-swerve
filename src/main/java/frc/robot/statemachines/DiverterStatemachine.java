package frc.robot.statemachines;

import frc.lib.state.StateMachine;

public class DiverterStatemachine extends StateMachine<DiverterStatemachine.DiverterState> {
    private DiverterState state = DiverterState.RETRACT;

    private void updateState(){
        switch (state) {
            default:
                break;
        }
    }

    @Override
    public void requestState(DiverterState state){

    }

    @Override
    public DiverterState getState(){
        return state;
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
    public boolean isDone(){
        switch(state){
            default:
                return true;
        }
    }

    @Override
    public boolean isDynamic() {
        switch(state){
            default:
                return true;
        }
    }

    public enum DiverterState {
        RETRACT(0.0,0.0),
        HANDOFF(0.0,0.0), //from shooter
        ALIGN_AMP(0.0,0.0),
        ALIGN_TRAP(0.0,0.0),
        PLACE_AMP(0.0,1.0),
        PLACE_TRAP(0.0,1.0);

        private Double position, speed;

        public Double getPosition(){
            return position;
        }

        public Double getSpeed(){
            return speed;
        }

        private DiverterState(Double position, Double speed){
            this.position = position;
            this.speed = speed;
        }
    }
}
