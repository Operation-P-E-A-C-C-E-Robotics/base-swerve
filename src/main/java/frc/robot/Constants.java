// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.lib.swerve.SwerveDescription.*;

public final class Constants {
  public static final class Swerve {
    public static final Dimensions dimensions = new Dimensions(0,0);

    public static final CANIDs frontLeftIDs = new CANIDs(0, 0, 0);
    public static final CANIDs frontRighIDs = new CANIDs(0, 0, 0);
    public static final CANIDs rearLeftIDs = new CANIDs(0, 0, 0);
    public static final CANIDs rearRightIDs = new CANIDs(0, 0, 0);

    public static final Gearing gearing = new Gearing(0, 0, 0, 0);
    public static final EncoderOffsets offsets = new EncoderOffsets(0, 0, 0, 0);
    public static final Inversion inversion = new Inversion(false, true);
    public static final Physics physics = new Physics(0, 0, 0, 0);
  
    public static final PidGains driveGains = new PidGains(0, 0, 0, 0, 0);
    public static final PidGains angleGains = new PidGains(0, 0, 0, 0, 0);
  
    public static final int pigeonCANId = 0;
    public static final boolean invertSteerMotors = false;
  }
}
