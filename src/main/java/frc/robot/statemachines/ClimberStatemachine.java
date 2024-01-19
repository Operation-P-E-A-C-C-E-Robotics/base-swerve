package frc.robot.statemachines;

import frc.lib.state.StateMachine;
import frc.robot.subsystems.Climber;

public class ClimberStatemachine extends StateMachine<ClimberStatemachine.ClimberState>{
    private ClimberState state = ClimberState.RETRACT;

    private final Climber climber;

    public ClimberStatemachine(Climber climber){
        this.climber = climber;
    }

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
    public void update(){
        //TODO climber observation
        updateState();
        switch(state) {
            default:
                break;            
        }
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
    public boolean isDynamic() {
        switch(state){
            default:
                return true;
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
