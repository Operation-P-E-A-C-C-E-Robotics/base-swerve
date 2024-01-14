package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;

public class TriggerIntakeStatemachine extends Command {
    private TriggerIntakeState state = TriggerIntakeState.RETRACT;

    private void updateState(){
        switch (state) {
            case AVOID:
                break;
            case EJECT:
                break;
            case INTAKE:
                break;
            case RETRACT:
                break;
            default:
                break;
        }
    }

    public void requestState(TriggerIntakeState state){

    }

    public TriggerIntakeState getState(){
        return state;
    }

    @Override
    public void execute(){
        updateState();
        switch(state) {
            case AVOID:
                break;
            case EJECT:
                break;
            case INTAKE:
                break;
            case RETRACT:
                break;
            default:
                break;
            
        }
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
