package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class DiverterStatemachine extends Command {
    private DiverterState state = DiverterState.RETRACT;

    private void updateState(){
        switch (state) {
            case ALIGN_AMP:
                break;
            case ALIGN_TRAP:
                break;
            case PLACE_AMP:
                break;
            case PLACE_TRAP:
                break;
            case RETRACT:
                break;
            default:
                break;
        }
    }

    public void requestState(DiverterState state){

    }

    @Override
    public void execute(){
        updateState();
        switch(state) {
            case ALIGN_AMP:
                break;
            case ALIGN_TRAP:
                break;
            case PLACE_AMP:
                break;
            case PLACE_TRAP:
                break;
            case RETRACT:
                break;
            default:
                break;            
        }
    }

    enum DiverterState {
        RETRACT(0.0,0.0),
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
