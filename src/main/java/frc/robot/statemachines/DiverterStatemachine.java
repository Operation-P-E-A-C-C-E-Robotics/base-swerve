package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.planners.StageAvoidancePlanner;
import frc.robot.subsystems.Diverter;

public class DiverterStatemachine extends StateMachine<DiverterStatemachine.DiverterState> {
    private DiverterState state = DiverterState.RETRACT;

    private final Diverter diverter;

    private StageAvoidancePlanner stageAvoidancePlanner;

    public DiverterStatemachine(Diverter diverter, StageAvoidancePlanner stageAvoidancePlanner){
        this.diverter = diverter;
        this.stageAvoidancePlanner = stageAvoidancePlanner;
    }

    private void updateState(){
        //automatically transition to aligning the trap once we have detected a handoff
        if(state == DiverterState.HANDOFF) {
            if(diverter.detectsNote()) state = DiverterState.ALIGN_TRAP;
        }

        if((state == DiverterState.ALIGN_AMP || state == DiverterState.PLACE_AMP) && stageAvoidancePlanner.shouldStow()) state = DiverterState.RETRACT;
    }

    @Override
    public void requestState(DiverterState state){
        switch (state) {
            //don't allow placing the amp unless we are already aligned with the amp
            case PLACE_AMP:
                if(this.state == DiverterState.ALIGN_AMP && isDone()) {
                    this.state = state;
                    break;
                }
                this.state = DiverterState.ALIGN_AMP;
                break;
            //don't allow placing the trap unless we are already aligned with the trap and have a note in the diverter
            case PLACE_TRAP:
                if(this.state == DiverterState.ALIGN_TRAP && isDone()) {
                    this.state = state;
                    break;
                }
                if(this.state == DiverterState.HANDOFF) {
                    this.state = DiverterState.ALIGN_TRAP;
                    break;
                }
                this.state = DiverterState.HANDOFF;
                break;
            //don't allow aligning the trap unless we have a note in the diverter
            case ALIGN_TRAP:
                if(this.state == DiverterState.HANDOFF) {
                    this.state = state;
                    break;
                }
                this.state = DiverterState.HANDOFF;
                break;
            default:
                break;
        }
    }

    @Override
    public DiverterState getState(){
        return state;
    }

    @Override
    public void update(){
        updateState();
        SmartDashboard.putString("Diverter State", state.name());
        diverter.setDiverterExtension(state.getPosition());
        diverter.setDiverterRoller(state.getSpeed());
    }

    @Override
    public boolean isDone(){
        return diverter.atSetpoint() && state.getSpeed() == 0 ? true : !diverter.detectsNote();
    }

    @Override
    public boolean isDynamic() {
        return state == DiverterState.HANDOFF;
    }

    public enum DiverterState {
        RETRACT(0.0,0.0),
        HANDOFF(0.0,0.0), //from shooter
        ALIGN_AMP(0.0,0.0),
        ALIGN_TRAP(0.0,0.0),
        PLACE_AMP(0.0,1.0),
        PLACE_TRAP(0.0,1.0),
        CLIMB(0.0,0.0);

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
