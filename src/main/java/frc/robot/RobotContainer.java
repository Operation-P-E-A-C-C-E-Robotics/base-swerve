// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.sensors.LimelightHelper;
import frc.robot.commands.PeaccyDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  /* OI CONSTANTS */
  private final int translationAxis = 3;
  private final int strafeAxis = 2;
  private final int rotationAxis = 0;

  /* SENSORS */
  LimelightHelper limelight = new LimelightHelper("limelight");

  /* SUBSYSTEMS */
  //ONE OF THESE MUST BE COMMENTED OUT. ONLY USE THE TUNEABLE ONE FOR TUNING.
  private final DriveTrain driveTrain = new DriveTrain(limelight);
  //private final DriveTrainTuneable driveTrainTuneable = new DriveTrainTuneable();

  /* OI DEFINITIONS */
  private final Joystick driverController = new Joystick(0);

  private final JoystickButton fieldCentricButton = new JoystickButton(driverController, 7);
  private final JoystickButton lockInButton = new JoystickButton(driverController, 8);
  private final JoystickButton zeroButton = new JoystickButton(driverController, 9);
  private final JoystickButton closedLoopButton = new JoystickButton(driverController, 5);

  /* COMMANDS */
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(
    () -> -driverController.getRawAxis(translationAxis),
    () -> -driverController.getRawAxis(strafeAxis),
    () -> -driverController.getRawAxis(rotationAxis),
    () -> (double) driverController.getPOV(),
    () -> driverController.getPOV() != -1,
    () -> !fieldCentricButton.getAsBoolean(),
    () -> closedLoopButton.getAsBoolean(),
    () -> lockInButton.getAsBoolean(),
    () -> zeroButton.getAsBoolean(),
    driveTrain
  );

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData("AUTO MODE", autoChooser);
  }

  private void configureBindings() {
    driveTrain.setDefaultCommand(peaccyDrive);
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
