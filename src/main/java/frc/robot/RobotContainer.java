// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SeanyDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  private final int translationAxis = 3;
  private final int strafeAxis = 2;
  private final int rotationAxis = 0;

  private final DriveTrain driveTrain = new DriveTrain();

  private final Joystick driverController = new Joystick(0);

  private final JoystickButton fieldCentricButton = new JoystickButton(driverController, 1);
  private final JoystickButton lockInButton = new JoystickButton(driverController, 2);
  private final Trigger autoAngleTrigger = new Trigger(() -> driverController.getPOV() != -1);

  private final SeanyDrive seanyDrive = new SeanyDrive(
    () -> -driverController.getRawAxis(translationAxis),
    () -> -driverController.getRawAxis(strafeAxis),
    () -> -driverController.getRawAxis(rotationAxis),
    () -> (double) driverController.getPOV(),
    () -> autoAngleTrigger.getAsBoolean(),
    () -> !fieldCentricButton.getAsBoolean(),
    () -> true,
    () -> lockInButton.getAsBoolean(),
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
