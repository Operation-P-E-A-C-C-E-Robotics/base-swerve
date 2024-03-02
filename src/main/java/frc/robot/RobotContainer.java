package frc.robot;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.telemetry.MultiTracers;
import frc.robot.auto.Autonomous;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.MotionPlanner;
import frc.robot.planners.NoteTracker;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.FlipperStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.TriggerIntakeStatemachine;
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
    private final MotionPlanner motionPlanner = new MotionPlanner();

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
    private final ShooterStatemachine shooterStatemachine = new ShooterStatemachine(shooter, aimPlanner, () -> {
        if(!shooter.flywheelAtTargetVelocity()) return false;
        if(!pivot.atSetpoint()) return false;
        if(!swerveStatemachine.transitioning()) return false;
        if((swerve.getChassisSpeeds().vxMetersPerSecond > 0.1 && swerve.getChassisSpeeds().vyMetersPerSecond > 0.1) && !OI.Inputs.enableShootWhileMoving.getAsBoolean()) return false;
        if(OI.Inputs.wantsPlace.getAsBoolean()) return false;
        return true;
    });
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

    public void zeroAutoHeading() {
        swerveStatemachine.zeroAutoHeading();
    }

    /**
     * The main update loop of the robot.
     * This is called periodically by the main robot class.
     * It updates the supersystem state, planners, and state machines.
     */
    public void run() {
        MultiTracers.trace("RobotContainer::run", "RobotContainer::run");
        /* UPDATE PLANNERS */
        motionPlanner.update();
        MultiTracers.trace("RobotContainer::run", "motionPlanner.update");
        aimPlanner.update();
        MultiTracers.trace("RobotContainer::run", "aimPlanner.update");

        /* TEST DASHBOARD */
        if(RobotState.isTest()) {
            return;
        }

        /* LET THE DRIVERS COOK */
        if(RobotState.isTeleop()) {
            // update with the state the driver wants
            teleopStatemachine.requestState(TeleopInputs.getInstance().getWantedTeleopState());
            MultiTracers.trace("RobotContainer::run", "teleopStatemachine.requestState");
            swerveStatemachine.requestState(TeleopInputs.getInstance().getWantedSwerveState());
            MultiTracers.trace("RobotContainer::run", "swerveStatemachine.requestState");

            //run all the state machines
            teleopStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "teleopStatemachine.update");
            swerveStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "swerveStatemachine.update");
            triggerIntakeStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "triggerIntakeStatemachine.update");
            pivotStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "pivotStatemachine.update");
            shooterStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "shooterStatemachine.update");
            // diverterStatemachine.update();
            // MultiTracers.trace("RobotContainer::run", "diverterStatemachine.update");
            // climberStatemachine.update();
            // MultiTracers.trace("RobotContainer::run", "climberStatemachine.update");
            
            // handle driver overrides
            TeleopInputs.getInstance().handleOverrides();
            MultiTracers.trace("RobotContainer::run", "TeleopInputs.getInstance().handleOverrides");
            NoteTracker.update(teleopStatemachine.getState());
            SmartDashboard.putString("Note tracket", NoteTracker.getLocation().name());
        }

        /* AUTONOMOUS */
        if(RobotState.isAutonomous()) {
            Autonomous.testAuto.run(swerveStatemachine, teleopStatemachine);
            teleopStatemachine.update();
            swerveStatemachine.update();
            // flywheelIntakeStatemachine.update();
            triggerIntakeStatemachine.update();
            pivotStatemachine.update();
            shooterStatemachine.update();
        }
        MultiTracers.print("RobotContainer::run");
    }
}
