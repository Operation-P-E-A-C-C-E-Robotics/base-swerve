// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.safety.Inspiration;
import frc.lib.telemetry.ControlSystemTelemetry;
import frc.robot.Constants.Core;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private PowerDistribution pdp = new PowerDistribution(Core.PDPCanId, Core.PDPModuleType);

  private boolean isInMatch = false; //are we at comp

  public Robot() {
    super(Constants.period);
    CommandScheduler.getInstance().setPeriod(Constants.period);
  }

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    //log data from network tables (SmartDashboard, etc.)
    DataLogManager.start();
    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(DataLogManager.getLog());

    //log current draw
    SmartDashboard.putData("PDP", pdp);
    
    //BRING THE HYPE
    isInMatch = Inspiration.initializeInspirationOpt1();
    if(isInMatch){
      Inspiration.inspireDriversInit();
    } else {
      Inspiration.inspireProgrammersInit();
    }
    
    System.out.println("Robot Initialized");
  }

  @Override
  public void robotPeriodic() {
    //run the robot
    CommandScheduler.getInstance().run();
    ControlSystemTelemetry.update(null);
    Inspiration.updateSlowPrinter(); //KEEP THE HYPE MACHINE ROLLIN
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
    Inspiration.inspireAutonomous(isInMatch);

    // schedule the autonomous command
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Inspiration.inspireTeleopInit(true);
    System.out.println("Robot Teleop");

    //stop autonomous
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
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
