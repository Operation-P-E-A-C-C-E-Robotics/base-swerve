package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelIntakeStatemachine extends Command {
    private FlywheelIntakeState state = FlywheelIntakeState.RETRACT;

    private void updateState(){
        switch (state) {
            default:
                break;
        }
    }

    public void requestState(FlywheelIntakeState state){

    }

    @Override
    public void execute(){
        updateState();
        switch(state) {
            default:
                break;
            
        }
    }

    public FlywheelIntakeState getState(){
        return state;
    }

    enum FlywheelIntakeState{
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

        private FlywheelIntakeState (Double deployAngle, Double speed){
            this.deployAngle = deployAngle;
            this.speed = speed;
        }
    }
}
