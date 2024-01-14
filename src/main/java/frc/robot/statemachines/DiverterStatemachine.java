package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;

public class DiverterStatemachine extends Command {
    private DiverterState state = DiverterState.RETRACT;

    private void updateState(){
        switch (state) {
            default:
                break;
        }
    }

    public void requestState(DiverterState state){

    }

    public DiverterState getState(){
        return state;
    }

    @Override
    public void execute(){
        updateState();
        switch(state) {
            default:
                break;            
        }
    }

    enum DiverterState {
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
