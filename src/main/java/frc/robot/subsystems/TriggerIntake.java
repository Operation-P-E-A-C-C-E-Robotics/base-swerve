package frc.robot.subsystems;

import static frc.robot.Constants.TriggerIntake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class TriggerIntake {

    private CANSparkMax deployCAN = new CANSparkMax(triggerIntakeDeployMotorId, MotorType.kBrushless);
    private CANSparkMax rollerCAN = new CANSparkMax(triggerIntakeRollerMotorId, MotorType.kBrushless);

    private RelativeEncoder deployEncoder = deployCAN.getEncoder();

    private SparkPIDController deployController = deployCAN.getPIDController();

    private double targetRotation = 0.0;



    public TriggerIntake () {

        rollerCAN.setInverted(triggerIntakeRollerMotorInverted);
        deployCAN.setInverted(triggerIntakeDeployMotorInverted);
        

        deployCAN.enableSoftLimit(SoftLimitDirection.kForward, true);

        deployController.setP(triggerIntakeDeployKp);
        deployController.setI(triggerIntakeDeployKi);
        deployController.setD(triggerIntakeDeployKp);

        deployEncoder.setPositionConversionFactor(triggerIntakeDeployGearing);
    }

    public void setDeploymentAngle (double angle) {
        //you can change your soft limits by changing deploy soft limits in constants.
        deployCAN.setSoftLimit(SoftLimitDirection.kForward, triggerIntakeDeployMaxAngle);
        deployCAN.setSoftLimit(SoftLimitDirection.kForward, triggerIntakeDeployMinAngle);

        targetRotation = angle; 
        deployController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void setRollerSpeed (double speed) {
        rollerCAN.set(speed);
    }

    public double getDeploymentAngle () {

        return deployEncoder.getPosition();
    }

    public boolean deployedToSetpoint () {
        return (targetRotation >= (getDeploymentAngle() - triggerIntakeDeployTolerance) &&
         targetRotation <= (getDeploymentAngle() - triggerIntakeDeployTolerance));
    }
}
