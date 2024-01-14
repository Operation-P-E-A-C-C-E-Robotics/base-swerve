package frc.robot.statemachines;

import frc.robot.statemachines.ClimberStatemachine.ClimberState;
import frc.robot.statemachines.DiverterStatemachine.DiverterState;
import frc.robot.statemachines.FlywheelIntakeStatemachine.FlywheelIntakeState;
import frc.robot.statemachines.PivotStatemachine.PivotState;
import frc.robot.statemachines.ShooterStatemachine.ShooterState;
import frc.robot.statemachines.TriggerIntakeStatemachine.TriggerIntakeState;

public class SupersystemStatemachine {
    

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
