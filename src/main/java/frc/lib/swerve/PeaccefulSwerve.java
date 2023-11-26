package frc.lib.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;

/**
 * OH MY LORD DONT MAKE ME EXTEND YOUR STUPID CLASS TO ADD BASIC FUNCTIONALITY.
 * ADD ONE FUNCTION SO I CAN GET THE GODDAMN KNEMATICS DONT MAKE THEM """"PROTECTED"""" SO I HAVE TO DO ALL THIS NONSENSE JUST TO GET THE
 * CHASSIS SPEEDS. 
 */
public class PeaccefulSwerve extends SwerveDrivetrain {
    public PeaccefulSwerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants[] modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
    }

    public PeaccefulSwerve(SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants[] modules, int pigeonCANId) {
        super(driveTrainConstants, modules);
    }
 
    public PeaccefulSwerve(SwerveDrivetrainConstants swerveConstants, SwerveModuleConstants frontLeft,
            SwerveModuleConstants frontRight, SwerveModuleConstants rearLeft, SwerveModuleConstants rearRight) {
        super(swerveConstants, frontLeft, frontRight, rearLeft, rearRight);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public double[] getAngleClosedLoopErrors() {
        double[] errors = new double[4];
        int i = 0;
        for (SwerveModule module : Modules) {
            errors[i] = module.getSteerMotor().getClosedLoopError().getValueAsDouble();
        }
        return errors;
    }

    public double[] getDriveClosedLoopErrors() {
        double[] errors = new double[4];
        int i = 0;
        for (SwerveModule module : Modules) {
            errors[i] = module.getDriveMotor().getClosedLoopError().getValueAsDouble();
        }
        return errors;
    }

    public void applyDriveConfigs (Slot0Configs configs) {
        for (SwerveModule module : Modules) {
            module.getDriveMotor().getConfigurator().apply(configs);
        }
    }

    public void applySteerConfigs (Slot0Configs configs) {
        for (SwerveModule module : Modules) {
            module.getSteerMotor().getConfigurator().apply(configs);
        }
    }

    public void setSteerCurrentLimit (double amps) {
        var limit = new CurrentLimitsConfigs();
        limit.StatorCurrentLimit = amps;
        limit.StatorCurrentLimitEnable = true;
        for (SwerveModule module : Modules) {
            module.getSteerMotor().getConfigurator().apply(limit);
        }
    }

    public double getModuleSteerCurrent(int i) {
        return Modules[i].getSteerMotor().getStatorCurrent().getValueAsDouble();
    }

    public double getModuleDriveCurrent(int i) {
        return Modules[i].getDriveMotor().getStatorCurrent().getValueAsDouble();
    }

    public double getModuleAngle(int i) {
        return Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble();
    }

    public double getModuleAngleError(int i) {
        return Modules[i].getSteerMotor().getClosedLoopError().getValueAsDouble();
    }

    public double getModuleRotationalRate(int i) {
        return Modules[i].getCANcoder().getVelocity().getValueAsDouble();
    }

    public double getModuleDriveError(int i) {
        return Modules[i].getDriveMotor().getClosedLoopError().getValueAsDouble();
    }

    /**
     * ONLY USE IN TEST MODE. WILL DO NOTHING IF NOT IN TEST MODE.
     * @param speed the speed to spin the angle motors at.
     */
    public void spinAngleMotors(double speed) {
        if(!RobotState.isTest()){
            System.err.println("WARNING (PeaccefulSwerve.spinAngleMotors): This method can only be used in test mode.");
            return;
        }
        setControl(new SwerveRequest.Idle());
        for (SwerveModule module : Modules) {
            module.getSteerMotor().set(speed);
        }
    }

}
