package frc.robot;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.EnumSendableChooser;
import frc.robot.RobotStatemachine.RobotState;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.CollisionAvoidancePlanner;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.FlipperStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.TriggerIntakeStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine.FlywheelIntakeState;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diverter;
import frc.robot.subsystems.FlywheelIntake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TriggerIntake;

/**
 * The container for all the subsystems and state machines of the robot.
 * This is where the bulk of the robot logic is stored and updated.
 */
public class RobotContainer {
    private static RobotContainer instance = null;

    /* SUBSYSTEMS */
    private final Swerve swerve = Swerve.getInstance();
    private final FlywheelIntake flywheelIntake = FlywheelIntake.getInstance();
    private final TriggerIntake triggerIntake = TriggerIntake.getInstance();
    private final Pivot pivot = Pivot.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Diverter diverter = Diverter.getInstance();
    private final Climber climber = Climber.getInstance();

    /* PLANNERS */
    private final CollisionAvoidancePlanner intakeMotionPlanner = new CollisionAvoidancePlanner(
        pivot::getPivotPosition,
        flywheelIntake::getDeploymentAngle,
        () -> swerve.getChassisSpeeds().vxMetersPerSecond
    );

    private final AimPlanner aimPlanner = new AimPlanner(
        () -> swerve.getPose(),
        () -> swerve.getChassisSpeeds(),
        () -> true
    );

    /* STATE MACHINES */
    private final SwerveStatemachine swerveStatemachine = new SwerveStatemachine(swerve, aimPlanner);
    private final FlywheelIntakeStatemachine flywheelIntakeStatemachine = new FlywheelIntakeStatemachine(flywheelIntake, intakeMotionPlanner, shooter::hasNote);
    private final TriggerIntakeStatemachine triggerIntakeStatemachine = new TriggerIntakeStatemachine(triggerIntake, intakeMotionPlanner, shooter::hasNote);
    private final PivotStatemachine pivotStatemachine = new PivotStatemachine(pivot, aimPlanner, intakeMotionPlanner);
    private final ShooterStatemachine shooterStatemachine = new ShooterStatemachine(shooter, aimPlanner, () -> false); //TODO
    private final FlipperStatemachine diverterStatemachine = new FlipperStatemachine(diverter);
    private final ClimberStatemachine climberStatemachine = new ClimberStatemachine(climber, () -> swerve.getGyroAngle().getX());

    private final RobotStatemachine robotStatemachine = new RobotStatemachine(
        swerveStatemachine,
        flywheelIntakeStatemachine,
        triggerIntakeStatemachine,
        shooterStatemachine,
        pivotStatemachine,
        diverterStatemachine,
        climberStatemachine,
        intakeMotionPlanner,
        aimPlanner
    );

    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();
        return instance;
    }

    public RobotStatemachine getRobotStatemachine() {
        return robotStatemachine;
    }

    public boolean hasNote() {
        return shooter.hasNote();
    }

    /**
     * The main update loop of the robot.
     * This is called periodically by the main robot class.
     * It updates the supersystem state, planners, and state machines.
     */
    public void run() {
        intakeMotionPlanner.update();
        aimPlanner.update();
        if(edu.wpi.first.wpilibj.RobotState.isTeleop()) updateTeleopControls();
        if(edu.wpi.first.wpilibj.RobotState.isTest()) {
            runTestDashboard();
            return;
        }
        robotStatemachine.update();
    }

    private void updateTeleopControls () {
        //EXAMPLE BUTTON:
        if(OI.DriveTrain.isLockIn.getAsBoolean()) {
            robotStatemachine.requestSwerveState(SwerveState.LOCK_IN);
        }
        else if(!OI.DriveTrain.isFieldRelative.getAsBoolean()) {
            robotStatemachine.requestSwerveState(SwerveState.ROBOT_CENTRIC);
        }
        else {
            robotStatemachine.requestSwerveState(OI.DriveTrain.isOpenLoop.getAsBoolean() ? SwerveState.OPEN_LOOP_TELEOP : SwerveState.CLOSED_LOOP_TELEOP);
        }
    }

    EnumSendableChooser<RobotStatemachine.RobotState> robotStateChooser = new EnumSendableChooser<>(RobotState.values());
    EnumSendableChooser<SwerveState> swerveStateChooser = new EnumSendableChooser<>(SwerveState.values());
    EnumSendableChooser<FlywheelIntakeState> flywheelIntakeStateChooser = new EnumSendableChooser<>(FlywheelIntakeState.values());
    EnumSendableChooser<TriggerIntakeStatemachine.TriggerIntakeState> triggerIntakeStateChooser = new EnumSendableChooser<>(TriggerIntakeStatemachine.TriggerIntakeState.values());
    EnumSendableChooser<PivotStatemachine.PivotState> pivotStateChooser = new EnumSendableChooser<>(PivotStatemachine.PivotState.values());
    EnumSendableChooser<ShooterStatemachine.ShooterState> shooterStateChooser = new EnumSendableChooser<>(ShooterStatemachine.ShooterState.values());
    EnumSendableChooser<FlipperStatemachine.FlipperState> diverterStateChooser = new EnumSendableChooser<>(FlipperStatemachine.FlipperState.values());
    EnumSendableChooser<ClimberStatemachine.ClimberState> climberStateChooser = new EnumSendableChooser<>(ClimberStatemachine.ClimberState.values());

    /* TEST MODE OVERRIDES */
    public void initTestDashboard () {
        SmartDashboard.putBoolean("Override Robot State", false);
        SmartDashboard.putBoolean("Override Swerve State", false);
        SmartDashboard.putBoolean("Override Flywheel Intake State", false);
        SmartDashboard.putBoolean("Override Trigger Intake State", false);
        SmartDashboard.putBoolean("Override Pivot State", false);
        SmartDashboard.putBoolean("Override Shooter State", false);
        SmartDashboard.putBoolean("Override Diverter State", false);
        SmartDashboard.putBoolean("Override Climber State", false);

        SmartDashboard.putData("Robot State Chooser", robotStateChooser);
        SmartDashboard.putData("Swerve State Chooser", swerveStateChooser);
        SmartDashboard.putData("Flywheel Intake State Chooser", flywheelIntakeStateChooser);
        SmartDashboard.putData("Trigger Intake State Chooser", triggerIntakeStateChooser);
        SmartDashboard.putData("Pivot State Chooser", pivotStateChooser);
        SmartDashboard.putData("Shooter State Chooser", shooterStateChooser);
        SmartDashboard.putData("Diverter State Chooser", diverterStateChooser);
        SmartDashboard.putData("Climber State Chooser", climberStateChooser);
    }

    public void runTestDashboard () {
        if(SmartDashboard.getBoolean("Override Robot State", false)) {
            robotStatemachine.requestState(robotStateChooser.getSelected());
        } 

        if(SmartDashboard.getBoolean("Override Swerve State", false)) {
            robotStatemachine.requestSwerveState(swerveStateChooser.getSelected());
        }

        robotStatemachine.update();

        if(SmartDashboard.getBoolean("Override Flywheel Intake State", false)) {
            flywheelIntakeStatemachine.updateWithState(flywheelIntakeStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Trigger Intake State", false)) {
            triggerIntakeStatemachine.updateWithState(triggerIntakeStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Pivot State", false)) {
            pivotStatemachine.updateWithState(pivotStateChooser.getSelected());
        }

        if(SmartDashboard.getBoolean("Override Shooter State", false)) {
            shooterStatemachine.updateWithState(shooterStateChooser.getSelected());
        }

         if(SmartDashboard.getBoolean("Override Diverter State", false)) {
             diverterStatemachine.updateWithState(diverterStateChooser.getSelected());
         }

         if(SmartDashboard.getBoolean("Override Climber State", false)) {
             climberStatemachine.updateWithState(climberStateChooser.getSelected());
         }
    }
}
