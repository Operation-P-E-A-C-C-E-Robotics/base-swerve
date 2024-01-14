package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberStatemachine extends Command {
    private ClimberState state = ClimberState.RETRACT;

    private void updateState(){
        switch (state) {
            case BALANCE:
                break;
            case EXTEND:
                break;
            case RETRACT:
                break;
            default:
                break;
        }
    }

    public void requestState(ClimberState state){

    }

    public ClimberState getState(){
        return state;
    }

    @Override
    public void execute(){
        updateState();
        switch(state) {
            case BALANCE:
                break;
            case EXTEND:
                break;
            case RETRACT:
                break;
            default:
                break;            
        }
    }

    enum ClimberState{
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
