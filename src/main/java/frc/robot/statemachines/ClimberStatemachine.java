package frc.robot.statemachines;

import frc.lib.state.StateMachine;

public class ClimberStatemachine extends StateMachine<ClimberStatemachine.ClimberState>{
    private ClimberState state = ClimberState.RETRACT;

    private void updateState(){
        switch (state) {
            default:
                break;
        }
    }

    @Override
    public void requestState(ClimberState state){

    }

    @Override
    public ClimberState getState(){
        return state;
    }

    @Override
    public boolean isDone(){
        switch(state){
            default:
                return true;
        }
    }

    @Override
    public void update(){
        updateState();
        switch(state) {
            default:
                break;            
        }
    }

    public enum ClimberState{
        //todo
        RETRACT(0.0, false),
        EXTEND(0.0, false),
        BALANCE(0.0, true);

        private Double position;
        private Boolean balance;

        public Double getPosition(){
            return position;
        }

        public Boolean isBalance(){
            return balance;
        }

        private ClimberState(Double position, Boolean balance){
            this.position = position;
            this.balance = balance;
        }
    }
}
