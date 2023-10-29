package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

public class DriveTrain {
    private static final SwerveDrivetrainConstants drivetrainConstants   = new SwerveDrivetrainConstants();
    private static final SwerveModuleConstantsFactory allModuleConstants = new SwerveModuleConstantsFactory();

    private static final SwerveModuleConstants  frontLeftConstants  = new SwerveModuleConstants(),
                                                frontRightConstants = new SwerveModuleConstants(),
                                                backLeftConstants   = new SwerveModuleConstants(),
                                                backRightConstants  = new SwerveModuleConstants();
}

