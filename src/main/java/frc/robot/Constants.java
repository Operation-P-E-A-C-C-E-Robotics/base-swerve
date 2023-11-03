// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.lib.swerve.SwerveDescription.CANIDs;
import frc.lib.swerve.SwerveDescription.Dimensions;
import frc.lib.swerve.SwerveDescription.EncoderOffsets;
import frc.lib.swerve.SwerveDescription.Gearing;
import frc.lib.swerve.SwerveDescription.Inversion;
import frc.lib.swerve.SwerveDescription.Physics;
import frc.lib.swerve.SwerveDescription.PidGains;

public final class Constants {
  public static final class Swerve {
    public static final Dimensions dimensions = new Dimensions(Units.inchesToMeters(24.75), Units.inchesToMeters(24.75));

    public static final CANIDs frontLeftIDs = new CANIDs(8, 10, 9); //module 3 / 2
    public static final CANIDs frontRighIDs = new CANIDs(5, 7, 6); //module 2 / 1
    public static final CANIDs rearLeftIDs = new CANIDs(11, 13, 12); //module 4 / 3
    public static final CANIDs rearRightIDs = new CANIDs(2, 4, 3); //module 1 / 0

    public static final Gearing gearing = new Gearing(DriveGearRatios.SDSMK4i_L2, ((150.0 / 7.0) / 1.0), 2.0, 0);
    public static final EncoderOffsets offsets = new EncoderOffsets(
      -0, 
      -0.227539, 
      -0.708496, 
      -0.407959
    );

    public static final Inversion inversion = new Inversion(true, true, false, true); //todo
    public static final Physics physics = new Physics(0.0001, 0.0001, 800, 10); //todo
  
    public static final PidGains driveGains = new PidGains(0.2, 0, 0, 0, 0); //todo
    public static final PidGains angleGains = new PidGains(70, 0, 0, 0, 0); //todo
  
    public static final int pigeonCANId = 14;
    public static final boolean invertSteerMotors = true;
  }

  public class DriveGearRatios{
    /* SDS MK3 */
    /** SDS MK3 - 8.16 : 1 */
    public static final double SDSMK3_Standard = (8.16 / 1.0);
    /** SDS MK3 - 6.86 : 1 */
    public static final double SDSMK3_Fast = (6.86 / 1.0);

    /* SDS MK4 */
    /** SDS MK4 - 8.14 : 1 */
    public static final double SDSMK4_L1 = (8.14 / 1.0);
    /** SDS MK4 - 6.75 : 1 */
    public static final double SDSMK4_L2 = (6.75 / 1.0);
    /** SDS MK4 - 6.12 : 1 */
    public static final double SDSMK4_L3 = (6.12 / 1.0);
    /** SDS MK4 - 5.14 : 1 */
    public static final double SDSMK4_L4 = (5.14 / 1.0);
    
    /* SDS MK4i */
    /** SDS MK4i - 8.14 : 1 */
    public static final double SDSMK4i_L1 = (8.14 / 1.0);
    /** SDS MK4i - 6.75 : 1 */
    public static final double SDSMK4i_L2 = (6.75 / 1.0);
    /** SDS MK4i - 6.12 : 1 */
    public static final double SDSMK4i_L3 = (6.12 / 1.0);
}
}
