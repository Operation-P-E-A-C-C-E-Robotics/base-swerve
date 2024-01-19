package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.DiverterStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.TriggerIntakeStatemachine;
import frc.robot.statemachines.ClimberStatemachine.ClimberState;
import frc.robot.statemachines.DiverterStatemachine.DiverterState;
import frc.robot.statemachines.FlywheelIntakeStatemachine.FlywheelIntakeState;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.statemachines.PivotStatemachine.PivotState;
import frc.robot.statemachines.ShooterStatemachine.ShooterState;
import frc.robot.statemachines.TriggerIntakeStatemachine.TriggerIntakeState;
import frc.robot.subsystems.FlywheelIntake;

public class RobotStatemachine extends StateMachine<RobotStatemachine.RobotState>{
    private RobotState state = RobotState.REST_WITHOUT_GAMEPIECE;

    public static FlywheelIntake flywheelIntake = new FlywheelIntake();
    
    private final SwerveStatemachine swerveStatemachine;
    private final FlywheelIntakeStatemachine flywheelIntakeStatemachine;
    private final TriggerIntakeStatemachine triggerIntakeStatemachine;
    private final ShooterStatemachine shooterStatemachine;
    private final PivotStatemachine pivotStatemachine;
    private final DiverterStatemachine diverterStatemachine;
    private final ClimberStatemachine climberStatemachine;
    
    private final SendableChooser<RobotState> simStateChooser = new SendableChooser<>();

    public RobotStatemachine (SwerveStatemachine swerveStatemachine, 
                            FlywheelIntakeStatemachine flywheelIntakeStatemachine, 
                            TriggerIntakeStatemachine triggerIntakeStatemachine, 
                            ShooterStatemachine shooterStatemachine, 
                            PivotStatemachine pivotStatemachine, 
                            DiverterStatemachine diverterStatemachine, 
                            ClimberStatemachine climberStatemachine) {
        this.swerveStatemachine = swerveStatemachine;
        this.flywheelIntakeStatemachine = flywheelIntakeStatemachine;
        this.triggerIntakeStatemachine = triggerIntakeStatemachine;
        this.shooterStatemachine = shooterStatemachine;
        this.pivotStatemachine = pivotStatemachine;
        this.diverterStatemachine = diverterStatemachine;
        this.climberStatemachine = climberStatemachine;

        if(Robot.isSimulation()) {
            simStateChooser.setDefaultOption("Rest Without Gamepiece", RobotState.REST_WITHOUT_GAMEPIECE);
            simStateChooser.addOption("Rest With Gamepiece", RobotState.REST_WITH_GAMEPIECE);
            simStateChooser.addOption("Intake Flywheel", RobotState.INTAKE_FLYWHEEL);
            simStateChooser.addOption("Intake Trigger", RobotState.INTAKE_TRIGGER);
            simStateChooser.addOption("Aim", RobotState.AIM);
            simStateChooser.addOption("Shoot", RobotState.SHOOT);
            simStateChooser.addOption("Align Amp", RobotState.ALIGN_AMP);
            simStateChooser.addOption("Place Amp", RobotState.PLACE_AMP);
            simStateChooser.addOption("Pre Climb", RobotState.PRE_CLIMB);
            simStateChooser.addOption("Climb Extend", RobotState.CLIMB_EXTEND);
            simStateChooser.addOption("Climb Retract", RobotState.CLIMB_RETRACT);
            simStateChooser.addOption("Handoff", RobotState.HANDOFF);
            simStateChooser.addOption("Align Trap", RobotState.ALIGN_TRAP);
            simStateChooser.addOption("Place Trap", RobotState.PLACE_TRAP);
            SmartDashboard.putData("Supersystem State", simStateChooser);
        }
    }

    /**
     * Request a state change.
     * Won't change state if the requested state results in a dangerous situation,
     * e.g. intaking while climbing.
     * @param state
     */
    @Override
    public void requestState(RobotState state){
        //Conditional state transitions:
        switch (this.state) {
            default:
                break;
        }

        //Automatic state transitions:
        switch (this.state) {
            default:
                break;
        }

        //illegal state transitions:
        switch (this.state) {
            default:
                break;
        }
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
        switch (state) {
            default:
                break;
        }
    }

    /**
     * Make the robot attain the desired state
     */
    @Override
    public void update(){
        if(Robot.isSimulation()) requestState(simStateChooser.getSelected());
        updateState();
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
        REST_WITHOUT_GAMEPIECE,
        REST_WITH_GAMEPIECE,
        INTAKE_FLYWHEEL (
            FlywheelIntakeState.INTAKE,
            TriggerIntakeState.RETRACT,
            ShooterState.INTAKE
        ),
        INTAKE_TRIGGER (
            FlywheelIntakeState.RETRACT,
            TriggerIntakeState.INTAKE,
            ShooterState.INTAKE
        ),
        AIM(
            ShooterState.AIM,
            PivotState.AIM
        ),
        SHOOT(
            ShooterState.SHOOT,
            PivotState.AIM
        ),
        //ADD SETPOINTS
        ALIGN_AMP(
            ShooterState.REST,
            PivotState.AMP,
            DiverterState.ALIGN_AMP
        ),
        PLACE_AMP(
            ShooterState.REST,
            PivotState.AMP,
            DiverterState.PLACE_AMP
        ),
        PRE_CLIMB(
            ShooterState.REST,
            PivotState.PRE_CLIMB
        ),
        CLIMB_EXTEND(
            TriggerIntakeState.AVOID,
            PivotState.CLIMB,
            DiverterState.RETRACT, //possible climb state
            ClimberState.EXTEND
        ),
        CLIMB_RETRACT(
            TriggerIntakeState.AVOID,
            PivotState.CLIMB,
            DiverterState.RETRACT,
            ClimberState.RETRACT
        ),
        HANDOFF(
            FlywheelIntakeState.RETRACT, 
            TriggerIntakeState.AVOID, 
            ShooterState.HANDOFF, 
            PivotState.CLIMB, 
            DiverterState.HANDOFF, 
            ClimberState.RETRACT
        ),
        ALIGN_TRAP(
            TriggerIntakeState.AVOID,
            PivotState.CLIMB,
            DiverterState.ALIGN_TRAP,
            ClimberState.RETRACT
        ),
        PLACE_TRAP(
            ShooterState.REST,
            PivotState.CLIMB,
            DiverterState.PLACE_TRAP
        );

        private FlywheelIntakeState flywheelIntakeState;
        private TriggerIntakeState triggerIntakeState;
        private ShooterState shooterState;
        private PivotState pivotState;
        private DiverterState diverterState;
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

        public DiverterState getDiverterState(){
            return diverterState;
        }

        public ClimberState getClimberState(){
            return climberState;
        }

        private RobotState (FlywheelIntakeState flywheelIntakeState,
                                TriggerIntakeState triggerIntakeState,
                                ShooterState shooterState,
                                PivotState pivotState,
                                DiverterState diverterState,
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
            this(flywheelIntakeState, triggerIntakeState, shooterState, PivotState.REST, DiverterState.RETRACT, ClimberState.RETRACT);
        }

        private RobotState(ShooterState shooterState, PivotState pivotState){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, shooterState, pivotState, DiverterState.RETRACT, ClimberState.RETRACT);
        }

        private RobotState(ShooterState shooterState, PivotState pivotState, DiverterState diverterState){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, shooterState, pivotState, diverterState, ClimberState.RETRACT);
        }

        private RobotState(TriggerIntakeState triggerIntakeState, PivotState pivotState, DiverterState diverterState, ClimberState climberState){
            this(FlywheelIntakeState.RETRACT, triggerIntakeState, ShooterState.REST, pivotState, diverterState, climberState);
        }

        private RobotState(){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, ShooterState.REST, PivotState.REST, DiverterState.RETRACT, ClimberState.RETRACT);
        }
    }
}
