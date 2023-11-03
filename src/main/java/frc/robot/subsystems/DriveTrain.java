package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveDescription;
import static frc.robot.Constants.Swerve.*;

public class DriveTrain extends SubsystemBase {
    private final SwerveDrivetrain swerve;

    private final Field2d field = new Field2d();

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
        swerve.tareEverything();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData(this);
        System.out.println("DriveTrain Initialized");
    }

    public void drive(SwerveRequest request) {
        swerve.setControl(request);
    }

    public Pose2d getPose () {
        if(swerve.odometryIsValid()) return swerve.getState().Pose;
        return new Pose2d();
    }

    @Override
    public void periodic() {
        field.setRobotPose(getPose());
        System.out.println(getPose());
    }

    @Override
    public void simulationPeriodic() {
        swerve.updateSimState(0.02, 12);
    }

    public void zeroOdometry() {
        swerve.tareEverything();
    }
}

