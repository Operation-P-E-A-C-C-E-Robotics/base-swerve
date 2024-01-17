// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.telemetry.ControlSystemTelemetry;
import frc.robot.Constants.ControlSystem;
import frc.robot.statemachines.RobotStatemachine;
import frc.robot.statemachines.RobotStatemachine.RobotState;

public class Robot extends TimedRobot {
  private PowerDistribution pdp = new PowerDistribution(ControlSystem.PDPCanId, ControlSystem.PDPModuleType);
  private Timer scheduleTimer = new Timer();

  RobotStatemachine supersystemStatemachine = new RobotStatemachine();

  public Robot() {
    super(Constants.period);
    SmartDashboard.updateValues();
  }

  @Override
  public void robotInit() {
    //log data from network tables (SmartDashboard, etc.)
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    DriverStation.startDataLog(DataLogManager.getLog());

    //log current draw
    SmartDashboard.putData("PDP", pdp);
    System.out.println("Robot Initialized");
    System.out.println("yay the software didn't crash yet");
  }

  @Override
  public void robotPeriodic() {
    if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
      supersystemStatemachine.requestState(RobotState.REST_WITHOUT_GAMEPIECE);
      return;
    }
    //run the robot
    scheduleTimer.reset();
    scheduleTimer.start();
    supersystemStatemachine.update();

    ControlSystemTelemetry.update(null, scheduleTimer.get());
  }

  @Override
  public void disabledInit() {
    DataLogManager.logNetworkTables(false);
    System.out.println("Robot Disabled");
    System.out.println("let's hope it doesn't move lol");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    DataLogManager.logNetworkTables(true);
    System.out.println("Robot Autonomous");
    System.out.println("EVERYBODY PANIC PEACCY IS RUNNING AUTONOMOUS AND HE DOESN'T KNOW WHAT HE'S DOING");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    DataLogManager.logNetworkTables(true);
    System.out.println("Robot Teleop");
    System.out.println("EVERYBODY RUN PEACCY IS DRIVING THE ROBOT AND HE WILL CRASH IT VERY SOON");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    DataLogManager.logNetworkTables(true);
    System.out.println("Robot Test");
    System.out.println("ok but what are yall doing i don't even know why you'd be running in test mode");
  }
  @Override
  public void testPeriodic() {}
  
  @Override
  public void simulationInit() {
    System.out.println("Robot Simulation");
    System.out.println("Simulation is stupid, good luck when you actually get a robot to test this on lol");
  }

  @Override
  public void simulationPeriodic() {}
}
