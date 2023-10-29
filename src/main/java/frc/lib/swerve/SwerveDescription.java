package frc.lib.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDescription {

    public static SwerveDrivetrain generateDrivetrain (
                                Dimensions dimensions, 
                                CANIDs frontLeftIDs,
                                CANIDs frontRightIDs,
                                CANIDs rearLeftIDs,
                                CANIDs rearRightIDs,
                                Gearing gearing,
                                EncoderOffsets offsets,
                                Inversion inversion,
                                Physics physics,
                                PidGains driveGains,
                                PidGains angleGains,
                                int pigeonCANId,
                                boolean invertSteerMotors){
        SwerveDrivetrainConstants swerveConstants = new SwerveDrivetrainConstants()
                                                    .withPigeon2Id(pigeonCANId)
                                                    .withSupportsPro(false)
                                                    .withCANbusName("rio");
        SwerveModuleConstantsFactory globalModuleConstants = new SwerveModuleConstantsFactory()
                                    .withDriveMotorGearRatio(gearing.driveRatio)
                                    .withSteerMotorGearRatio(gearing.steerRatio)
                                    .withWheelRadius(gearing.wheelRadius)
                                    .withSlipCurrent(physics.wheelSlipCurrent)
                                    .withSteerMotorGains(angleGains)
                                    .withDriveMotorGains(driveGains)
                                    .withSpeedAt12VoltsMps(physics.freeSpeed)
                                    .withSteerInertia(physics.angularInertia)
                                    .withDriveInertia(physics.linearInertia)
                                    .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
                                    .withCouplingGearRatio(gearing.steerCouplingRatio)
                                    .withSteerMotorInverted(invertSteerMotors);

    
        SwerveModuleConstants frontLeft = globalModuleConstants.createModuleConstants(
            frontLeftIDs.steerMotor, 
            frontLeftIDs.driveMotor, 
            frontLeftIDs.encoder, 
            offsets.frontLeft,
            dimensions.frontLeft.getX(), 
            dimensions.frontLeft.getY(), 
            inversion.frontLeft
        );

        SwerveModuleConstants frontRight = globalModuleConstants.createModuleConstants(
            frontRightIDs.steerMotor, 
            frontRightIDs.driveMotor, 
            frontRightIDs.encoder, 
            offsets.frontRight,
            dimensions.frontRight.getX(), 
            dimensions.frontRight.getY(), 
            inversion.frontRight
        );

        SwerveModuleConstants rearLeft = globalModuleConstants.createModuleConstants(
            rearLeftIDs.steerMotor, 
            rearLeftIDs.driveMotor, 
            rearLeftIDs.encoder, 
            offsets.rearLeft,
            dimensions.rearLeft.getX(), 
            dimensions.rearLeft.getY(), 
            inversion.rearLeft
        );

        SwerveModuleConstants rearRight = globalModuleConstants.createModuleConstants(
            rearRightIDs.steerMotor, 
            rearRightIDs.driveMotor, 
            rearRightIDs.encoder, 
            offsets.rearRight,
            dimensions.rearRight.getX(), 
            dimensions.rearRight.getY(), 
            inversion.rearRight
        );

        return new SwerveDrivetrain(
            swerveConstants,
            frontLeft,
            frontRight,
            rearLeft,
            rearRight
        );
    }

    public static class Dimensions {
        public final double width, height;
        public final Translation2d frontLeft, frontRight, rearLeft, rearRight;

        public Dimensions (double width, double height) {
            this.width = width;
            this.height = height;

            frontLeft = new Translation2d(height/2, width/2);
            frontRight = new Translation2d(height/2, -width/2);
            rearLeft = new Translation2d(-height/2, width/2);
            rearRight = new Translation2d(-height/2, -width/2);

        }
    }

    public static class Physics {
        public final double linearInertia, angularInertia, wheelSlipCurrent, freeSpeed;

        public Physics (double linearInertia, double angularInertia, double wheelSlipCurrent, double freeSpeed) {
            this.linearInertia = linearInertia;
            this.angularInertia = angularInertia;
            this.wheelSlipCurrent = wheelSlipCurrent;
            this.freeSpeed = freeSpeed;
        }
    }
    
    public static class EncoderOffsets {
        public final double frontLeft, frontRight, rearLeft, rearRight;

        public EncoderOffsets (double frontLeft, double frontRight, double rearLeft, double rearRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.rearLeft = rearLeft;
            this.rearRight = rearRight;
        }
    }

    public static class Inversion {
        public final boolean frontLeft, frontRight, rearLeft, rearRight;

        public Inversion (boolean frontLeft, boolean frontRight, boolean rearLeft, boolean rearRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.rearLeft = rearLeft;
            this.rearRight = rearRight;
        }

        public Inversion (boolean left, boolean right) {
            this.frontLeft = left;
            this.frontRight = right;
            this.rearLeft = left;
            this.rearRight = right;
        }
    }

    public static class Gearing {
        public final double driveRatio, steerRatio, wheelRadius, steerCouplingRatio;

        public Gearing (double driveRatio, double steerRatio, double wheelRadius, double steerCouplingRatio) {
            this.driveRatio = driveRatio;
            this.steerRatio = steerRatio;
            this.wheelRadius = wheelRadius;
            this.steerCouplingRatio = steerCouplingRatio;
        }
    }

    public static class CANIDs {
        public final int driveMotor,steerMotor,encoder;
        public CANIDs (int driveMotor, int steerMotor, int encoder) {
            this.driveMotor = driveMotor;
            this.steerMotor = steerMotor;
            this.encoder = encoder;
        }
    }

    public static class PidGains extends Slot0Configs {
        public PidGains(double kP, double kI, double kD, double kV, double kS) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kS = kS;
        }
  }
}
