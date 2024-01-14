package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotStatemachine extends Command{
    private PivotState state = PivotState.REST;

    private void updateState(){
        switch (state) {
            case AMP:
                break;
            case AIM:
                break;
            case PRE_CLIMB:
                break;
            case CLIMB:
                break;
            case INTAKE:
                break;
            case REST:
                break;
            default:
                break;
        }
    }

    public void requestState(PivotState state){

    }

    public PivotState getState(){
        return state;
    }

    @Override
    public void execute(){
        updateState();
        switch(state) {
            case AMP:
                break;
            case AIM:
                break;
            case PRE_CLIMB:
                break;
            case CLIMB:
                break;
            case INTAKE:
                break;
            case REST:
                break;
            default:
                break;            
        }
    }

    enum PivotState {
        REST(0.0),
        INTAKE(0.0),
        AMP(0.0),
        PRE_CLIMB(0.0),
        CLIMB(0.0),
        //add shooter setpoints
        AIM(0.0);

        private Double angle;

        public Double getAngle(){
            return angle;
        }

        private PivotState (Double angle){
            this.angle = angle;
        }
    }
}
