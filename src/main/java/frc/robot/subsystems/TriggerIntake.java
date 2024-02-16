package frc.robot.subsystems;

import static frc.robot.Constants.TriggerIntake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.Util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class TriggerIntake {

    private CANSparkMax deployMotor = new CANSparkMax(triggerIntakeDeployMotorId, MotorType.kBrushless);
    private CANSparkMax rollerMotor = new CANSparkMax(triggerIntakeRollerMotorId, MotorType.kBrushless);

    private RelativeEncoder deployEncoder = deployMotor.getEncoder();

    private SparkPIDController deployController = deployMotor.getPIDController();

    private double targetRotation = 0.0;

    private TriggerIntake () {
        rollerMotor.setInverted(triggerIntakeRollerMotorInverted);
        deployMotor.setInverted(triggerIntakeDeployMotorInverted);
        
        deployMotor.setSoftLimit(SoftLimitDirection.kForward, triggerIntakeDeployMaxAngle);
        deployMotor.setSoftLimit(SoftLimitDirection.kReverse, triggerIntakeDeployMinAngle);
        deployMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        deployMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

        deployController.setP(triggerIntakeDeployKp);
        deployController.setI(triggerIntakeDeployKi);
        deployController.setD(triggerIntakeDeployKp);

        deployEncoder.setPositionConversionFactor(triggerIntakeDeployGearing);
    }

    public void setDeploymentAngle (Rotation2d angle) {
        targetRotation = angle.getRotations(); 
        deployController.setReference(angle.getRotations(), CANSparkMax.ControlType.kPosition);
    }

    public void setRollerSpeed (double speed) {
        rollerMotor.set(speed);
    }

    public Rotation2d getDeploymentAngle () {
        return Rotation2d.fromRotations(deployEncoder.getPosition());
    }

    public boolean deployedToSetpoint () {
        return Util.inRange(getDeploymentAngle().getRotations() - targetRotation, triggerIntakeDeployTolerance);
    }

    private static final TriggerIntake instance = new TriggerIntake();
    public static TriggerIntake getInstance(){
        return instance;
    }
}
