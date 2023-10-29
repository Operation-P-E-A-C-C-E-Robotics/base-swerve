// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    System.out.println("Robot Initialized");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot Disabled");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    System.out.println("Robot Autonomous");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    System.out.println("Robot Teleop");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    System.out.println("Robot Test");
    CommandScheduler.getInstance().cancelAll();
  }
  @Override
  public void testPeriodic() {}
  
  @Override
  public void simulationInit() {
    System.out.println("Robot Simulation");
  }

  @Override
  public void simulationPeriodic() {}
}
