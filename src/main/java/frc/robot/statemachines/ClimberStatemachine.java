package frc.robot.statemachines;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.state.StateMachine;

public class ClimberStatemachine extends StateMachine<ClimberStatemachine.ClimberState>{
    private ClimberState state = ClimberState.RETRACT;

    private static ClimberObservation observation = new ClimberObservation();

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

    public static ClimberObservation getObservation(){
        return observation;
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

    public static class ClimberObservation{
        public final double extension;
        public final Pose3d pose;

        public ClimberObservation(){
            extension = 0.0;
            pose = new Pose3d();
        }

        public ClimberObservation(double extension, Pose3d pose){
            this.extension = extension;
            this.pose = pose;
        }
    }
}
