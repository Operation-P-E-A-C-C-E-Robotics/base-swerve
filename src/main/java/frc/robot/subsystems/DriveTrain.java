package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.RealSwerveDrivetrain;
import frc.lib.swerve.SwerveDescription;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.*;

public class DriveTrain extends SubsystemBase {
    private final RealSwerveDrivetrain swerve;

    private final Field2d field = new Field2d();

    private final SwerveRequest.ApplyChassisSpeeds autonomousRequest = new SwerveRequest.ApplyChassisSpeeds();

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

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getChassisSpeeds, 
            this::drive, 
            Constants.Swerve.pathFollowerConfig, 
            this
        );

        swerve.registerTelemetry((SwerveDriveState state) -> {
            field.setRobotPose(state.Pose);
            SmartDashboard.putNumber("Front Left Module Angle", state.ModuleStates[0].angle.getDegrees());
            SmartDashboard.putNumber("Front Right Module Angle", state.ModuleStates[1].angle.getDegrees());
            SmartDashboard.putNumber("Rear Left Module Angle", state.ModuleStates[2].angle.getDegrees());
            SmartDashboard.putNumber("Rear Right Module Angle", state.ModuleStates[3].angle.getDegrees());
            SmartDashboard.putNumber("Front Left Module Speed", state.ModuleStates[0].speedMetersPerSecond);
            SmartDashboard.putNumber("Front Right Module Speed", state.ModuleStates[1].speedMetersPerSecond);
            SmartDashboard.putNumber("Rear Left Module Speed", state.ModuleStates[2].speedMetersPerSecond);
            SmartDashboard.putNumber("Rear Right Module Speed", state.ModuleStates[3].speedMetersPerSecond);
        });

        SmartDashboard.putData("Field", field);

        System.out.println("DriveTrain Initialized");
    }

    public void drive(SwerveRequest request) {
        swerve.setControl(request);
    }

    public void drive(ChassisSpeeds speeds) {
        drive(autonomousRequest.withSpeeds(speeds));
    }

    public Pose2d getPose () {
        if(swerve.odometryIsValid()) return swerve.getState().Pose;
        return new Pose2d();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return swerve.getChassisSpeeds();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        swerve.updateSimState(0.02, 12);
    }

    public void resetOdometry() {
        swerve.tareEverything();
    }

    public void resetOdometry(Pose2d pose) {
        swerve.seedFieldRelative(pose);
    }
}

