package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelIntake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.Util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

//software for the intake that is on the front of the robot
public class FlywheelIntake {
    private CANSparkMax deployMotor = new CANSparkMax(flywheelIntakeDeployMotorId, MotorType.kBrushless);
    private CANSparkMax rollerMotor = new CANSparkMax(flywheelIntakeRollerMotorId, MotorType.kBrushless);

    private RelativeEncoder deployEncoder = deployMotor.getEncoder();

    private SparkPIDController deployPIDController = deployMotor.getPIDController();

    private double targetRotations = 0;

    private FlywheelIntake () {
        rollerMotor.setInverted(flywheelIntakeRollerMotorInverted);
        deployMotor.setInverted(flywheelIntakeDeployMotorInverted);

        //«««Warning»»» : dont let stretch interact with this code by any means. It has so much hatred 
        //for this code that it will break its own neck without warning
        deployMotor.setSoftLimit(SoftLimitDirection.kForward, flywheelIntakeDeployMaxAngle);
        deployMotor.setSoftLimit(SoftLimitDirection.kReverse, flywheelIntakeDeployMinAngle);
        deployMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        deployMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        deployPIDController.setP(flywheelIntakeDeployKp);   //Hehe, thats funny, it spells PID.... WAIT
        deployPIDController.setI(flywheelIntakeDeployKi);
        deployPIDController.setD(flywheelIntakeDeployKd);

        deployEncoder.setPositionConversionFactor(flywheelIntakeDeployGearing);
    }

    
    /**
     * set the target angle for the intake to deploy to, in rotations
     */
    public void setDeploymentAngle (Rotation2d angle) {
        targetRotations = angle.getRotations(); 
        deployPIDController.setReference(angle.getRotations(), CANSparkMax.ControlType.kPosition); //might need to divide by gearing, not sure.
    }

    /**
     * set the speed of the roller, from -1 to 1
     */
    public void setRollerSpeed (double speed) {
        rollerMotor.set(speed);
    }

    /**
     * get the current angle of the intake, in rotations
     */
    public Rotation2d getDeploymentAngle(){
        return Rotation2d.fromRotations(deployEncoder.getPosition());
    }
    /**
     * get whether the deploy motor is at its setpoint
     */

    public boolean deployedToSetpoint () {
        return Util.inRange(getDeploymentAngle().getRotations() - targetRotations, flywheelIntakeDeployTolerance);
    }

    private static FlywheelIntake instance = new FlywheelIntake();
    public static FlywheelIntake getInstance(){
        return instance;
    }
}