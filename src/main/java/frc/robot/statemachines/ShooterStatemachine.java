package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterStatemachine extends Command{
    private ShooterState state = ShooterState.REST;

    private void updateState(){
        switch (state) {
            case AUTOAIM:
                break;
            case AUTOSHOOT:
                break;
            case FIRE:
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

    @Override
    public void execute(){
        updateState();
        switch(state) {
            case AUTOAIM:
                break;
            case AUTOSHOOT:
                break;
            case FIRE:
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
        INTAKE(-0.0,0.0),
        INDEX(0.0,-0.0),
        //add shooter setpoints
        AUTOAIM,
        FIRE,
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
