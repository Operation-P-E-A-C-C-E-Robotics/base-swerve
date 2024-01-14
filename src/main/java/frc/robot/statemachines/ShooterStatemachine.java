package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterStatemachine extends Command{
    private ShooterState state = ShooterState.REST;

    private void updateState(){
        switch (state) {
            case AIM:
                break;
            case AUTOSHOOT:
                break;
            case SHOOT:
                break;
            case INDEX:
                break;
            case INTAKE:
                break;
            case REST:
                break;
            default:
                break;
        }
    }

    public void requestState(ShooterState state){

    }

    public ShooterState getState(){
        return state;
    }

    @Override
    public void execute(){
        updateState();
        switch(state) {
            case AIM:
                break;
            case AUTOSHOOT:
                break;
            case SHOOT:
                break;
            case INDEX:
                break;
            case INTAKE:
                break;
            case REST:
                break;
            default:
                break;            
        }
    }

    enum ShooterState{
        REST(0.0,0.0),
        INTAKE(-0.0,0.0), //NOTE: this should fold flat if the flywheel-side intake is out
        INDEX(0.0,-0.0),
        //add shooter setpoints
        AIM,
        SHOOT,
        AUTOSHOOT;

        private Double flywheelVelocity, triggerPercentage;

        public Double getFlywheelVelocity(){
            return flywheelVelocity;
        }

        public Double getTriggerPercentage(){
            return triggerPercentage;
        }

        private ShooterState (Double flywheelVelocity, Double triggerPercentage){
            this.flywheelVelocity = flywheelVelocity;
            this.triggerPercentage = triggerPercentage;
        }

        private ShooterState() {
            flywheelVelocity = Double.NaN;
            triggerPercentage = Double.NaN;
        }
    }
}
