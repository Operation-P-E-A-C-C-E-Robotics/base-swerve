package frc.robot.statemachines;

import java.util.function.BooleanSupplier;

import frc.lib.state.StateMachine;
import frc.robot.planners.AimPlanner;
import frc.robot.subsystems.Shooter;

public class ShooterStatemachine extends StateMachine<ShooterStatemachine.ShooterState> {
    private ShooterState state = ShooterState.REST;

    private ShooterState lastAimingState = ShooterState.AIM;

    private final Shooter shooter;
    private final AimPlanner aimPlanner;
    private final BooleanSupplier alignedToShoot;

    public ShooterStatemachine(Shooter shooter, AimPlanner aimPlanner, BooleanSupplier alignedToShoot){
        this.shooter = shooter;
        this.aimPlanner = aimPlanner;
        this.alignedToShoot = alignedToShoot;
    }

    /**
     * Handle the logic for changing states
     * e.g. intaking to indexing when the gamepiece is detected
     */
    private void updateState(){
        switch (state) {
            case REST:
                if(shooter.hasNote() && !(shooter.flywheelBeamBroken() && shooter.triggerBeamBroken())) state = ShooterState.INDEX;
            case INTAKE:
                if(shooter.flywheelBeamBroken()) state = ShooterState.INDEX;
            case SHOOT:
                if(!alignedToShoot.getAsBoolean()) state = lastAimingState;
            case INDEX:
                if(shooter.triggerBeamBroken() && shooter.flywheelBeamBroken()) state = ShooterState.REST;
            default:
                break;
        }
        if (
              state == ShooterState.AIM
            ||state == ShooterState.NEAR_SPEAKER_SETPOINT
            ||state == ShooterState.PROTECTED_SETPOINT
            ||state == ShooterState.CENTERLINE_SETPOINT
            && alignedToShoot.getAsBoolean() 
            && shooter.flywheelAtTargetVelocity()
        ) {
            lastAimingState = state;
            state = ShooterState.SHOOT;
        }
    }

    /**
     * Request a state for the mechanism to attain
     * Won't allow for a state that would result in a collision or other dangerous situation
     * e.g. changing state before we have finished INDEXing
     * @param state
     */
    @Override
    public void requestState(ShooterState state){
        this.state = state;
    }

    /**
     * make the mechanism attain the desired state
     */
    @Override
    public void update(){
        updateState();
        if(state == ShooterState.AIM) {
            shooter.setFlywheelVelocity(aimPlanner.getTargetFlywheelVelocity());
            return;
        }

        if(state == ShooterState.SHOOT) {
            if(lastAimingState == ShooterState.AIM) shooter.setFlywheelVelocity(aimPlanner.getTargetFlywheelVelocity());
            else shooter.setFlywheelVelocity(lastAimingState.getFlywheelVelocity());
            shooter.setTrigerPercent(state.getTriggerPercent());
            return;
        }

        if(state == ShooterState.INDEX){
            if(shooter.flywheelBeamBroken() && !shooter.triggerBeamBroken()) shooter.setTrigerPercent(-state.getTriggerPercent());
            else if (shooter.triggerBeamBroken() && !shooter.flywheelBeamBroken()) shooter.setTrigerPercent(state.getTriggerPercent());
            else shooter.setTrigerPercent(0.0);
        }

        shooter.setFlywheelVelocity(state.getFlywheelVelocity());
        shooter.setTrigerPercent(state.getTriggerPercent());
    }

    @Override
    public ShooterState getState(){
        return state;
    }

    @Override
    public boolean isDone(){
        return shooter.flywheelAtTargetVelocity();
    }

    @Override
    public boolean isDynamic() {
        return true;
    }

    public enum ShooterState{
        REST(0.0,0.0),
        INTAKE(0.0,0.0), //NOTE: this should fold flat if the flywheel-side intake is out
        INDEX(0.0,0.2),
        HANDOFF(0.0,0.0), //to diverter
        NEAR_SPEAKER_SETPOINT(0.0,0.0),
        PROTECTED_SETPOINT(0.0,0.0),
        CENTERLINE_SETPOINT(0.0,0.0),
        AIM(0.0,0.0),
        SHOOT(0.0,1.0);

        private Double flywheelVelocity, triggerPercent;

        public Double getFlywheelVelocity(){
            return flywheelVelocity;
        }

        public Double getTriggerPercent(){
            return triggerPercent;
        }

        private ShooterState (Double flywheelVelocity, Double triggerPercentage){
            this.flywheelVelocity = flywheelVelocity;
            this.triggerPercent = triggerPercentage;
        }

        private ShooterState() {
            flywheelVelocity = Double.NaN;
            triggerPercent = Double.NaN;
        }
    }
}
