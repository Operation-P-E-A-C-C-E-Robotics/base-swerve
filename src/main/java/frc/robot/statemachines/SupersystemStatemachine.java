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

    public void requestState(SupersystemState state){

    }

    private void updateState(){
        switch (state) {
            case REST_WITHOUT_GAMEPIECE:
                break;
            case REST_WITH_GAMEPIECE:
                break;
            case INTAKE_FLYWHEEL:
                break;
            case INTAKE_TRIGGER:
                break;
            case AIM:
                break;
            case SHOOT:
                break;
            case ALIGN_AMP:
                break;
            case PLACE_AMP:
                break;
            case PRE_CLIMB:
                break;
            case CLIMB_EXTEND:
                break;
            case CLIMB_RETRACT:
                break;
            case ALIGN_TRAP:
                break;
            case PLACE_TRAP:
                break;
            default:
                break;
        }
    }

    public SupersystemState getState(){
        return state;
    }

    public static FlywheelIntakeState getFlywheelIntakeState(){
        return flywheelIntakeStatemachine.getState();
    }

    public static TriggerIntakeState getTriggerIntakeState(){
        return triggerIntakeStatemachine.getState();
    }

    public static ShooterState getShooterState(){
        return shooterStatemachine.getState();
    }

    public static PivotState getPivotState(){
        return pivotStatemachine.getState();
    }

    public static DiverterState getDiverterState(){
        return diverterStatemachine.getState();
    }

    public static ClimberState getClimberState(){
        return climberStatemachine.getState();
    }

    public void execute(){
        updateState();
        flywheelIntakeStatemachine.requestState(state.getFlywheelIntakeState());
        triggerIntakeStatemachine.requestState(state.getTriggerIntakeState());
        shooterStatemachine.requestState(state.getShooterState());
        pivotStatemachine.requestState(state.getPivotState());
        diverterStatemachine.requestState(state.getDiverterState());
        climberStatemachine.requestState(state.getClimberState());
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
            PivotState.CLIMB,
            DiverterState.RETRACT, //possible climb state
            ClimberState.EXTEND
        ),
        CLIMB_RETRACT(
            PivotState.CLIMB,
            DiverterState.RETRACT,
            ClimberState.RETRACT
        ),
        ALIGN_TRAP(
            PivotState.AMP,
            DiverterState.ALIGN_TRAP,
            ClimberState.RETRACT
        ),
        PLACE_TRAP(
            ShooterState.REST,
            PivotState.AMP,
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

        private SupersystemState(PivotState pivotState, DiverterState diverterState, ClimberState climberState){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, ShooterState.REST, pivotState, diverterState, climberState);
        }

        private SupersystemState(){
            this(FlywheelIntakeState.RETRACT, TriggerIntakeState.RETRACT, ShooterState.REST, PivotState.REST, DiverterState.RETRACT, ClimberState.RETRACT);
        }
    }
}
