package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
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
    }

    @Override
    public void simulationPeriodic() {
        swerve.updateSimState(0.02, 12);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(
            ".default",
            () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
            null);
        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(
            ".command",
            () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
            null);
        builder.addDoubleProperty(
            "module 0 angle", 
            () -> getModuleAngleDeg(0),
            null    
        );
        builder.addDoubleProperty(
            "module 0 speed", 
            () -> getModuleSpeed(0),
            null    
        );
        builder.addDoubleProperty(
            "module 1 angle", 
            () -> getModuleAngleDeg(1),
            null    
        );
        builder.addDoubleProperty(
            "module 1 speed", 
            () -> getModuleSpeed(1),
            null    
        );
        builder.addDoubleProperty(
            "module 2 angle", 
            () -> getModuleAngleDeg(2),
            null    
        );
        builder.addDoubleProperty(
            "module 2 speed", 
            () -> getModuleSpeed(2),
            null    
        );
        builder.addDoubleProperty(
            "module 3 angle", 
            () -> getModuleAngleDeg(3),
            null    
        );
        builder.addDoubleProperty(
            "module 3 speed", 
            () -> getModuleSpeed(3),
            null    
        );
        builder.addBooleanProperty("odometry valid", 
            swerve::odometryIsValid, 
            null
        );
    }

    public double getModuleAngleDeg (int i) {
        if (i < 0 || i > 3) return 0;
        // if (!swerve.odometryIsValid()) return 0;
        return swerve.getModule(i).getCurrentState().angle.getDegrees();
    }
    public double getModuleSpeed (int i) {
        if (i < 0 || i > 3) return 0;
        // if (!swerve.odometryIsValid()) return 0;
        return swerve.getModule(i).getCurrentState().speedMetersPerSecond;
    }
}

