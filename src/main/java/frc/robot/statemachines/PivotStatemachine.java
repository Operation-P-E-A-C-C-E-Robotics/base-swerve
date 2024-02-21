package frc.robot.statemachines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.MotionPlanner;
import frc.robot.subsystems.Pivot;

public class PivotStatemachine extends StateMachine<PivotStatemachine.PivotState> {
    private PivotState state = PivotState.REST;

    private final Pivot pivot;
    private final AimPlanner aimPlanner;
    private final MotionPlanner intakeMotionPlanner;

    public PivotStatemachine(Pivot pivot, AimPlanner aimPlanner, MotionPlanner intakeMotionPlanner){
        this.pivot = pivot;
        this.aimPlanner = aimPlanner;
        this.intakeMotionPlanner = intakeMotionPlanner;

        SmartDashboard.putNumber("pivot sim debug", 0);
    }

    /**
     * Request a state for the mechanism to attain
     * Won't allow for a state that would result in a collision or other dangerous situation
     * e.g. flattening to intake before the intake is extended
     * @param state
     */
    @Override
    public void requestState(PivotState state){
        this.state = state;
    }

    /**
     * Make the mechanism attain the desired state
     */
    @Override
    public void update(){
        // if (state == PivotState.INTAKE && !intakeMotionPlanner.canFlattenPivot()) state = PivotState.REST;

        // if(state == PivotState.AUTO_AIM) {
        //     var angle = aimPlanner.getTargetPivotAngle();
        //     if(angle.getRadians() > MotionPlanner.interferenceLowerPivotAngle.getRadians() || intakeMotionPlanner.canFlattenPivot()) {
        //         pivot.setPivotPosition(angle);
        //         return;
        //     } else {
        //         state = PivotState.REST;
        //     }
        // }

        SmartDashboard.putString("Pivot State", state.name());
        pivot.setPivotPosition(state.getAngle());
    }

    @Override
    public PivotState getState(){
        return state;
    }
    
    @Override
    public boolean transitioning() {
       return !pivot.atSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return state == PivotState.INTAKE || state == PivotState.AUTO_AIM;
    }

    public enum PivotState {
        REST(Rotation2d.fromDegrees(25)),
        INTAKE(Rotation2d.fromDegrees(30)),
        AMP(Rotation2d.fromDegrees(30)),
        PRE_CLIMB(Rotation2d.fromDegrees(30)),
        CLIMB(Rotation2d.fromDegrees(30)),
        AIM_LAYUP(Rotation2d.fromDegrees(49)),
        AIM_PROTECTED(Rotation2d.fromDegrees(30)),
        AUTO_AIM(Rotation2d.fromDegrees(30));

        private Rotation2d angle;

        public Rotation2d getAngle(){
            return angle;
        }

        private PivotState (Rotation2d angle){
            this.angle = angle;
        }
    }
}
