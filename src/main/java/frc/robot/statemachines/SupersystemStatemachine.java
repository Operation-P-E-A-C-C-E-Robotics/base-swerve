package frc.robot.statemachines;

import frc.robot.statemachines.ClimberStatemachine.ClimberState;
import frc.robot.statemachines.DiverterStatemachine.DiverterState;
import frc.robot.statemachines.FlywheelIntakeStatemachine.FlywheelIntakeState;
import frc.robot.statemachines.PivotStatemachine.PivotState;
import frc.robot.statemachines.ShooterStatemachine.ShooterState;
import frc.robot.statemachines.TriggerIntakeStatemachine.TriggerIntakeState;

public class SupersystemStatemachine {
    private SupersystemState state = SupersystemState.REST_WITHOUT_GAMEPIECE;
    
    private static final FlywheelIntakeStatemachine flywheelIntakeStatemachine = new FlywheelIntakeStatemachine();
    private static final TriggerIntakeStatemachine triggerIntakeStatemachine = new TriggerIntakeStatemachine();
    private static final ShooterStatemachine shooterStatemachine = new ShooterStatemachine();
    private static final PivotStatemachine pivotStatemachine = new PivotStatemachine();
    private static final DiverterStatemachine diverterStatemachine = new DiverterStatemachine();
    private static final ClimberStatemachine climberStatemachine = new ClimberStatemachine();

    /**
     * Request a state change.
     * Won't change state if the requested state results in a dangerous situation,
     * e.g. intaking while climbing.
     * @param state
     */
    public void requestState(SupersystemState state){

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
    public void execute(){
        updateState();
        flywheelIntakeStatemachine.requestState(state.getFlywheelIntakeState());
        triggerIntakeStatemachine.requestState(state.getTriggerIntakeState());
        shooterStatemachine.requestState(state.getShooterState());
        pivotStatemachine.requestState(state.getPivotState());
        diverterStatemachine.requestState(state.getDiverterState());
        climberStatemachine.requestState(state.getClimberState());
    }

    /**
     * Get the state that is currently being
     * requested by the state machine
     */
    public SupersystemState getState(){
        return state;
    }

    /**
     * Get the state that the flywheel intake is currently executing
     */
    public static FlywheelIntakeState getFlywheelIntakeState(){
        return flywheelIntakeStatemachine.getState();
    }

    /**
     * Get the state that the trigger intake is currently executing
     */
    public static TriggerIntakeState getTriggerIntakeState(){
        return triggerIntakeStatemachine.getState();
    }

    /**
     * Get the state that the shooter is currently executing
     */
    public static ShooterState getShooterState(){
        return shooterStatemachine.getState();
    }

    /**
     * Get the state that the pivot is currently executing
     */
    public static PivotState getPivotState(){
        return pivotStatemachine.getState();
    }

    /**
     * Get the state that the diverter is currently executing
     */
    public static DiverterState getDiverterState(){
        return diverterStatemachine.getState();
    }

    /**
     * Get the state that the climber is currently executing
     */
    public static ClimberState getClimberState(){
        return climberStatemachine.getState();
    }
    

    enum SupersystemState {
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
        ),; 

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

        private SupersystemState (FlywheelIntakeState flywheelIntakeState,
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

        private SupersystemState(FlywheelIntakeState flywheelIntakeState,
                                TriggerIntakeState triggerIntakeState,
                                ShooterState shooterState) {
            this(flywheelIntakeState, triggerIntakeState, shooterState, PivotState.REST, DiverterState.RETRACT, ClimberState.RETRACT);
        }

        private SupersystemState(ShooterState shooterState, PivotState pivotState){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, shooterState, pivotState, DiverterState.RETRACT, ClimberState.RETRACT);
        }

        private SupersystemState(ShooterState shooterState, PivotState pivotState, DiverterState diverterState){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, shooterState, pivotState, diverterState, ClimberState.RETRACT);
        }

        private SupersystemState(TriggerIntakeState triggerIntakeState, PivotState pivotState, DiverterState diverterState, ClimberState climberState){
            this(FlywheelIntakeState.RETRACT, triggerIntakeState, ShooterState.REST, pivotState, diverterState, climberState);
        }

        private SupersystemState(){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, ShooterState.REST, PivotState.REST, DiverterState.RETRACT, ClimberState.RETRACT);
        }
    }
}
