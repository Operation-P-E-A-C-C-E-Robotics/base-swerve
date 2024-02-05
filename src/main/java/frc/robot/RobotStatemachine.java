package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.CollisionAvoidancePlanner;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.FlipperStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.TriggerIntakeStatemachine;
import frc.robot.statemachines.ClimberStatemachine.ClimberState;
import frc.robot.statemachines.FlipperStatemachine.FlipperState;
import frc.robot.statemachines.FlywheelIntakeStatemachine.FlywheelIntakeState;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.statemachines.PivotStatemachine.PivotState;
import frc.robot.statemachines.ShooterStatemachine.ShooterState;
import frc.robot.statemachines.TriggerIntakeStatemachine.TriggerIntakeState;

public class RobotStatemachine extends StateMachine<RobotStatemachine.RobotState>{
    private RobotState state = RobotState.REST;
    
    private final SwerveStatemachine swerveStatemachine;
    private final FlywheelIntakeStatemachine flywheelIntakeStatemachine;
    private final TriggerIntakeStatemachine triggerIntakeStatemachine;
    private final ShooterStatemachine shooterStatemachine;
    private final PivotStatemachine pivotStatemachine;
    private final FlipperStatemachine diverterStatemachine;
    private final ClimberStatemachine climberStatemachine;

    private final CollisionAvoidancePlanner intakeMotionPlanner;
    private final AimPlanner aimPlanner;
    //private final StageAvoidancePlanner stageAvoidancePlanner;

    public RobotStatemachine (SwerveStatemachine swerveStatemachine, 
                            FlywheelIntakeStatemachine flywheelIntakeStatemachine, 
                            TriggerIntakeStatemachine triggerIntakeStatemachine, 
                            ShooterStatemachine shooterStatemachine, 
                            PivotStatemachine pivotStatemachine, 
                            FlipperStatemachine diverterStatemachine, 
                            ClimberStatemachine climberStatemachine,
                            CollisionAvoidancePlanner intakeMotionPlanner,
                            AimPlanner aimPlanner) {
        this.swerveStatemachine = swerveStatemachine;
        this.flywheelIntakeStatemachine = flywheelIntakeStatemachine;
        this.triggerIntakeStatemachine = triggerIntakeStatemachine;
        this.shooterStatemachine = shooterStatemachine;
        this.pivotStatemachine = pivotStatemachine;
        this.diverterStatemachine = diverterStatemachine;
        this.climberStatemachine = climberStatemachine;
        this.intakeMotionPlanner = intakeMotionPlanner;
        this.aimPlanner = aimPlanner;
        // this.stageAvoidancePlanner = stageAvoidancePlanner;
    }

    /**
     * Request a state change.
     * Won't change state if the requested state results in a dangerous situation,
     * e.g. intaking while climbing.
     * @param state
     */
    @Override
    public void requestState(RobotState state){
        this.state = state;
    }

    /**
     * The drivetrain is it's own thing lol
     * @param state
     */
    public void requestSwerveState(SwerveState state){
        swerveStatemachine.requestState(state);
    }

    /**
     * handle automatic state transitions,
     * e.g. intaking flywheel-side to intaking trigger-side when the robot
     * changes direction
     */
    private void updateState(){
    }

    /**
     * Make the robot attain the desired state
     */
    @Override
    public void update(){
        updateState();

        SmartDashboard.putString("Robot State", state.name());

        if (state == RobotState.AUTO_AIM || state == RobotState.SHOOT) {
            swerveStatemachine.requestState(SwerveState.AIM);
        }

        swerveStatemachine.update();
        flywheelIntakeStatemachine.updateWithState(state.getFlywheelIntakeState());
        triggerIntakeStatemachine.updateWithState(state.getTriggerIntakeState());
        shooterStatemachine.updateWithState(state.getShooterState());
        pivotStatemachine.updateWithState(state.getPivotState());
        diverterStatemachine.updateWithState(state.getDiverterState());
        climberStatemachine.updateWithState(state.getClimberState());
    }

    /**
     * Get the state that is currently being
     * requested by the state machine
     */
    @Override
    public RobotState getState(){
        return state;
    }

    /**
     * Check if the robot has finished attaining the desired state
     */
    @Override
    public boolean isDone(){
        if(!flywheelIntakeStatemachine.isDone()) return false;
        if(!triggerIntakeStatemachine.isDone()) return false;
        if(!shooterStatemachine.isDone()) return false;
        if(!pivotStatemachine.isDone()) return false;
        if(!diverterStatemachine.isDone()) return false;
        if(!climberStatemachine.isDone()) return false;
        if(!swerveStatemachine.isDone()) return false;
        return true;
    }
    

    public enum RobotState {
        REST,
        INTAKE,
        INTAKE_FLYWHEEL (
            FlywheelIntakeState.INTAKE,
            TriggerIntakeState.RETRACT,
            ShooterState.INTAKE,
            PivotState.INTAKE
        ),
        INTAKE_TRIGGER (
            FlywheelIntakeState.RETRACT,
            TriggerIntakeState.INTAKE,
            ShooterState.INTAKE
        ),
        AIM_LAYUP (
            ShooterState.AIM_LAYUP,
            PivotState.AIM_LAYUP
        ),
        AIM_PROTECTED (
            ShooterState.AIM_PROTECTED,
            PivotState.AIM_PROTECTED
        ),
        AUTO_AIM(
            ShooterState.AUTO_AIM,
            PivotState.AUTO_AIM
        ),
        SHOOT(
            ShooterState.SHOOT,
            PivotState.AUTO_AIM
        ),
        ALIGN_AMP(
            ShooterState.RAMP_DOWN,
            PivotState.AMP,
            FlipperState.ALIGN_AMP
        ),
        PLACE_AMP(
            ShooterState.RAMP_DOWN,
            PivotState.AMP,
            FlipperState.PLACE_AMP
        ),
        PRE_CLIMB(
            ShooterState.RAMP_DOWN,
            PivotState.PRE_CLIMB
        ),
        CLIMB_EXTEND(
            TriggerIntakeState.AVOID,
            PivotState.CLIMB,
            FlipperState.RETRACT, //possible climb state
            ClimberState.EXTEND
        ),
        CLIMB_RETRACT(
            TriggerIntakeState.AVOID,
            PivotState.CLIMB,
            FlipperState.RETRACT,
            ClimberState.RETRACT
        ),
        HANDOFF(
            FlywheelIntakeState.RETRACT, 
            TriggerIntakeState.AVOID, 
            ShooterState.HANDOFF, 
            PivotState.CLIMB, 
            FlipperState.HANDOFF, 
            ClimberState.RETRACT
        ),
        ALIGN_TRAP(
            TriggerIntakeState.AVOID,
            PivotState.CLIMB,
            FlipperState.ALIGN_TRAP,
            ClimberState.RETRACT
        ),
        PLACE_TRAP(
            ShooterState.RAMP_DOWN,
            PivotState.CLIMB,
            FlipperState.PLACE_TRAP
        );

        private FlywheelIntakeState flywheelIntakeState;
        private TriggerIntakeState triggerIntakeState;
        private ShooterState shooterState;
        private PivotState pivotState;
        private FlipperState diverterState;
        private ClimberState climberState;

        public FlywheelIntakeState getFlywheelIntakeState() {
            return flywheelIntakeState;
        }

        public TriggerIntakeState getTriggerIntakeState(){
            return triggerIntakeState;
        }

        public ShooterState getShooterState(){
            return shooterState;
        }

        public PivotState getPivotState(){
            return pivotState;
        }

        public FlipperState getDiverterState(){
            return diverterState;
        }

        public ClimberState getClimberState(){
            return climberState;
        }

        private RobotState (FlywheelIntakeState flywheelIntakeState,
                                TriggerIntakeState triggerIntakeState,
                                ShooterState shooterState,
                                PivotState pivotState,
                                FlipperState diverterState,
                                ClimberState climberState){
            this.flywheelIntakeState = flywheelIntakeState;
            this.triggerIntakeState = triggerIntakeState;
            this.shooterState = shooterState;
            this.pivotState = pivotState;
            this.diverterState = diverterState;
            this.climberState = climberState;
        }

        private RobotState(FlywheelIntakeState flywheelIntakeState,
                                TriggerIntakeState triggerIntakeState,
                                ShooterState shooterState) {
            this(flywheelIntakeState, triggerIntakeState, shooterState, PivotState.REST, FlipperState.RETRACT, ClimberState.RETRACT);
        }

        private RobotState(FlywheelIntakeState flywheelIntakeState,
                                TriggerIntakeState triggerIntakeState,
                                ShooterState shooterState,
                                PivotState pivotState) {
            this(flywheelIntakeState, triggerIntakeState, shooterState, pivotState, FlipperState.RETRACT, ClimberState.RETRACT);
        }

        private RobotState(ShooterState shooterState, PivotState pivotState){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, shooterState, pivotState, FlipperState.RETRACT, ClimberState.RETRACT);
        }

        private RobotState(ShooterState shooterState, PivotState pivotState, FlipperState diverterState){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, shooterState, pivotState, diverterState, ClimberState.RETRACT);
        }

        private RobotState(TriggerIntakeState triggerIntakeState, PivotState pivotState, FlipperState diverterState, ClimberState climberState){
            this(FlywheelIntakeState.RETRACT, triggerIntakeState, ShooterState.RAMP_DOWN, pivotState, diverterState, climberState);
        }

        private RobotState(){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, ShooterState.RAMP_DOWN, PivotState.REST, FlipperState.RETRACT, ClimberState.RETRACT);
        }
    }
}
