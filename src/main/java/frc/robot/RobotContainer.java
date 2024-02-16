package frc.robot;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.EnumSendableChooser;
import frc.robot.TeleopStatemachine.TeleopState;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.MotionPlanner;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.FlipperStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.TriggerIntakeStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine.FlywheelIntakeState;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.statemachines.TriggerIntakeStatemachine.TriggerIntakeState;
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
    private final MotionPlanner motionPlanner = new MotionPlanner(
        pivot::getPivotPosition,
        flywheelIntake::getDeploymentAngle,
        () -> swerve.getChassisSpeeds().vxMetersPerSecond
    );

    private final AimPlanner aimPlanner = new AimPlanner(
        () -> swerve.getPose(),
        () -> swerve.getChassisSpeeds(),
        OI.Inputs.enableShootWhileMoving
    );

    /* STATE MACHINES */
    private final SwerveStatemachine swerveStatemachine = new SwerveStatemachine(swerve, aimPlanner);
    private final FlywheelIntakeStatemachine flywheelIntakeStatemachine = new FlywheelIntakeStatemachine(flywheelIntake, motionPlanner);
    private final TriggerIntakeStatemachine triggerIntakeStatemachine = new TriggerIntakeStatemachine(triggerIntake, motionPlanner);
    private final PivotStatemachine pivotStatemachine = new PivotStatemachine(pivot, aimPlanner, motionPlanner);
    private final ShooterStatemachine shooterStatemachine = new ShooterStatemachine(shooter, aimPlanner, () -> false); //TODO
    private final FlipperStatemachine diverterStatemachine = new FlipperStatemachine(diverter, motionPlanner);
    private final ClimberStatemachine climberStatemachine = new ClimberStatemachine(climber, () -> swerve.getGyroAngle().getX());

    private final TeleopStatemachine teleopStatemachine = new TeleopStatemachine(
        swerveStatemachine,
        flywheelIntakeStatemachine,
        triggerIntakeStatemachine,
        shooterStatemachine,
        pivotStatemachine,
        diverterStatemachine,
        climberStatemachine,
        motionPlanner,
        aimPlanner
    );

    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();
        return instance;
    }

    public TeleopStatemachine getTeleopStatemachine() {
        return teleopStatemachine;
    }

    Tracer tracer = new Tracer();

    /**
     * The main update loop of the robot.
     * This is called periodically by the main robot class.
     * It updates the supersystem state, planners, and state machines.
     */
    public void run() {
        tracer.addEpoch("RobotContainer.run(): start");
        motionPlanner.update();
        tracer.addEpoch("motionPlanner.update()");
        aimPlanner.update();
        tracer.addEpoch("aimPlanner.update()");

        if(RobotState.isTest()) {
            runTestDashboard();
            tracer.addEpoch("runTestDashboard()");
            return;
        }
        if(RobotState.isTeleop()) {
            teleopStatemachine.requestState(TeleopInputs.getInstance().getWantedTeleopState());
            swerveStatemachine.requestState(TeleopInputs.getInstance().getWantedSwerveState());


            teleopStatemachine.update();
            tracer.addEpoch("teleopStatemachine.update()");

            swerveStatemachine.update();
            tracer.addEpoch("swerveStatemachine.update()");


            flywheelIntakeStatemachine.update();
            // tracer.addEpoch("flywheelIntakeStatemachine.update()");

            triggerIntakeStatemachine.update();
            // tracer.addEpoch("triggerIntakeStatemachine.update()");

            // pivotStatemachine.update();
            // tracer.addEpoch("pivotStatemachine.update()");

            // shooterStatemachine.update();
            // tracer.addEpoch("shooterStatemachine.update()");

            // diverterStatemachine.update();
            // tracer.addEpoch("diverterStatemachine.update()");
            
            // climberStatemachine.update();
            // tracer.addEpoch("climberStatemachine.update()");

            if(OI.Overrides.forceTrigger.getAsBoolean()) {
                shooter.setTrigerPercent(1);
            }
            if(OI.Overrides.eject.getAsBoolean()) {
                flywheelIntake.setRollerSpeed(-1);
                triggerIntake.setRollerSpeed(-1);
                shooter.setTrigerPercent(-1);
                shooter.setFlywheelPercent(1);
                diverter.setDiverterRoller(1);
            }
        }
        if(RobotState.isAutonomous()) {
            //autoStatemachine.update();
        }
        tracer.printEpochs();
    }

    EnumSendableChooser<TeleopStatemachine.TeleopState> robotStateChooser = new EnumSendableChooser<>(TeleopState.values());
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
            teleopStatemachine.requestState(robotStateChooser.getSelected());
        } 

        if(SmartDashboard.getBoolean("Override Swerve State", false)) {
            teleopStatemachine.requestSwerveState(swerveStateChooser.getSelected());
        }

        teleopStatemachine.update();

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
