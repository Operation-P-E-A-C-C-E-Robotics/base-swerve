// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.sensors.LimelightHelper;
import frc.robot.commands.PeaccyDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  /* OI CONSTANTS */
  private final int translationAxis = 5;
  private final int strafeAxis = 4;
  private final int rotationAxis = 0;

  /* SENSORS */
  LimelightHelper limelight = new LimelightHelper("limelight");

  /* SUBSYSTEMS */
  //ONE OF THESE MUST BE COMMENTED OUT. ONLY USE THE TUNEABLE ONE FOR TUNING.
  private final DriveTrain driveTrain = new DriveTrain(limelight);
  // private final DriveTrainTuneable_old driveTrainTuneable = new DriveTrainTuneable();

  /* OI DEFINITIONS */
  private final Joystick driverController = new Joystick(0);
  
  private final JoystickButton zeroButton = new JoystickButton(driverController, 7); //for debugging
  private final JoystickButton closedLoopButton = new JoystickButton(driverController, 5);
  private final JoystickButton driveFallbackButton = new JoystickButton(driverController, 7);
  private final JoystickButton driveFallbackResetButton = new JoystickButton(driverController, 8);


  /* COMMANDS */
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(driveTrain);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData("AUTO MODE", autoChooser);
  }

  private void configureBindings() {
    peaccyDrive.withTranslation(() -> -driverController.getRawAxis(translationAxis))
               .withStrafe     (() -> -driverController.getRawAxis(strafeAxis))
               .withRotation   (() -> -driverController.getRawAxis(rotationAxis))
               .withHeading    (() -> (double) -driverController.getPOV())
               .useHeading     (() -> driverController.getPOV() != -1)
               .isFieldRelative(() -> driverController.getRawAxis(2) < 0.2)
               .isLockIn       (() -> driverController.getRawAxis(3) > 0.2)
               .isZeroOdometry (() -> zeroButton.getAsBoolean());
    driveTrain.setDefaultCommand(peaccyDrive);
    driveFallbackButton.onTrue(new InstantCommand(peaccyDrive::fallback, driveTrain)); //TODO it's annoying that we have to state requiements, could cause problems, find better way
    driveFallbackResetButton.onTrue(new InstantCommand(peaccyDrive::resetFallback, driveTrain));
    // new JoystickButton(driverController,1).onTrue(new InstantCommand(
    //   () -> {
    //     Command test = driveTrain.driveToPose(new Pose2d());
    //     test.schedule();
    //   }, driveTrain));
  }


  public Command getAutonomousCommand() {
    return AutoBuilder.followPathWithEvents(PathPlannerPath.fromPathFile("Example Path"));
  }
}
