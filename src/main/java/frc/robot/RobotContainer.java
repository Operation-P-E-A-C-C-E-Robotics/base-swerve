// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.PeaccyDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  private final int translationAxis = 3;
  private final int strafeAxis = 2;
  private final int rotationAxis = 0;

  private final DriveTrain driveTrain = new DriveTrain();

  private final Joystick driverController = new Joystick(0);

  private final JoystickButton fieldCentricButton = new JoystickButton(driverController, 7);
  private final JoystickButton lockInButton = new JoystickButton(driverController, 8);
  private final JoystickButton zeroButton = new JoystickButton(driverController, 9);
  private final JoystickButton closedLoopButton = new JoystickButton(driverController, 5);

  private final PeaccyDrive seanyDrive = new PeaccyDrive(
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

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveTrain.setDefaultCommand(seanyDrive);
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
