package frc.robot;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.IntakeMotionPlanner;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.DiverterStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.TriggerIntakeStatemachine;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diverter;
import frc.robot.subsystems.FlywheelIntake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TriggerIntake;

public class RobotContainer {
    private static RobotContainer instance = null;

    /* SUBSYSTEMS */
    private final Swerve swerve = new Swerve();
    private final FlywheelIntake flywheelIntake = new FlywheelIntake();
    private final TriggerIntake triggerIntake = new TriggerIntake();
    private final Pivot pivot = new Pivot();
    private final Shooter shooter = new Shooter();
    private final Diverter diverter = new Diverter();
    private final Climber climber = new Climber();

    /* PLANNERS */
    IntakeMotionPlanner intakeMotionPlanner = new IntakeMotionPlanner(
        pivot::getPivotPosition,
        flywheelIntake::getDeploymentAngle
    );

    AimPlanner aimPlanner = new AimPlanner(
        swerve::getPose,
        swerve::getChassisSpeeds,
        false
    );

    /* STATE MACHINES */
    private final SwerveStatemachine swerveStatemachine = new SwerveStatemachine(swerve, aimPlanner);
    private final FlywheelIntakeStatemachine flywheelIntakeStatemachine = new FlywheelIntakeStatemachine(flywheelIntake, intakeMotionPlanner);
    private final TriggerIntakeStatemachine triggerIntakeStatemachine = new TriggerIntakeStatemachine(triggerIntake, intakeMotionPlanner);
    private final PivotStatemachine pivotStatemachine = new PivotStatemachine(pivot, aimPlanner);
    private final ShooterStatemachine shooterStatemachine = new ShooterStatemachine(shooter, aimPlanner);
    private final DiverterStatemachine diverterStatemachine = new DiverterStatemachine(diverter);
    private final ClimberStatemachine climberStatemachine = new ClimberStatemachine(climber);

    /* Joysticks */
    private final Joystick driverJoystick = new Joystick(0);
    // private final Joystick operatorJoystick = new Joystick(1);

    private final RobotStatemachine robotStatemachine = new RobotStatemachine(
        swerveStatemachine,
        flywheelIntakeStatemachine,
        triggerIntakeStatemachine,
        shooterStatemachine,
        pivotStatemachine,
        diverterStatemachine,
        climberStatemachine
    );

    public RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    public RobotStatemachine getRobotStatemachine() {
        return robotStatemachine;
    }

    public void run() {
        if(RobotState.isTeleop()) updateTeleopControls();
        robotStatemachine.update();
    }

    private void updateTeleopControls () {
        //EXAMPLE BUTTON:
        // if(driverJoystick.getRawButtonPressed(0)) {
        //     robotStatemachine.requestSwerveState(SwerveState.ROBOT_CENTRIC);
        // }
    }
}
