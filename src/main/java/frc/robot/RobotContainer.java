// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.vision.Limelight;
import frc.robot.commands.drivetrain.PeaccyDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  /* OI CONSTANTS */
  private final int translationAxis = 5; //forward/backward
  private final int strafeAxis = 4;
  private final int rotationAxis = 0;
  private final int zeroButtonNo = 7;
  private final int fallbackButtonNo = 7;
  private final int fallbackResetButtonNo = 8;

  /* SENSORS */
  Limelight limelight = new Limelight("limelight");

  /* SUBSYSTEMS */
  //ONE OF THESE MUST BE COMMENTED OUT. ONLY USE THE TUNEABLE ONE FOR TUNING.
  private final DriveTrain driveTrain = new DriveTrain(limelight);
  // private final DriveTrainTuner driveTrainTuneable = new DriveTrainTuner();

  /* OI DEFINITIONS */
  private final Joystick driverController = new Joystick(0);
  
  private final JoystickButton zeroButton = new JoystickButton(driverController, zeroButtonNo); //for debugging
  private final JoystickButton driveFallbackButton = new JoystickButton(driverController, fallbackButtonNo);
  private final JoystickButton driveFallbackResetButton = new JoystickButton(driverController, fallbackResetButtonNo);


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
               .isFieldRelative(() -> driverController.getRawAxis(2) < 0.2) //left trigger
               .isLockIn       (() -> driverController.getRawAxis(3) > 0.2) //right trigger
               .isZeroOdometry (() -> zeroButton.getAsBoolean())
               .isOpenLoop(() -> false);
    driveTrain.setDefaultCommand(peaccyDrive);
    driveFallbackButton.onTrue(new InstantCommand(peaccyDrive::fallback, driveTrain)); 
    driveFallbackResetButton.onTrue(new InstantCommand(peaccyDrive::resetFallback, driveTrain));
  }


  public Command getAutonomousCommand() {
    return AutoBuilder.followPathWithEvents(PathPlannerPath.fromPathFile("Example Path"));
  }
}
