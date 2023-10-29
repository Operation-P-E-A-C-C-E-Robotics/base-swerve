// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

public final class Constants {
  public static final class Swerve {
    //velocity controller gains:
    private static final SwervePidGains steerGains = new SwervePidGains(50, 0, 0.05, 0,0);
    private static final SwervePidGains driveGains = new SwervePidGains(3, 0, 0, 0, 0);

  }

  static class SwervePidGains extends Slot0Configs {
    public SwervePidGains(double kP, double kI, double kD, double kV, double kS) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kV = kV;
      this.kS = kS;
      }
  }
}
