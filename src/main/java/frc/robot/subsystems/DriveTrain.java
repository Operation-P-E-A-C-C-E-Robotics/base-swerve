package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveDescription;
import static frc.robot.Constants.Swerve.*;

public class DriveTrain extends SubsystemBase {
    private final SwerveDrivetrain swerve;

    public DriveTrain() {
        swerve = SwerveDescription.generateDrivetrain(
            dimensions, 
            frontLeftIDs, 
            frontRighIDs, 
            rearLeftIDs, 
            rearRightIDs, 
            gearing, 
            offsets, 
            inversion, 
            physics, 
            driveGains, 
            angleGains, 
            pigeonCANId, 
            invertSteerMotors
        );
    }

    public void drive(SwerveRequest request) {
        swerve.setControl(request);
    }

    public Pose2d getPose () {
        return swerve.getState().Pose;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}

