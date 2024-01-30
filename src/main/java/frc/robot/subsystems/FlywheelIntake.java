package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelIntake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

//software for the intake that is on the front of the robot
public class FlywheelIntake {
    private CANSparkMax deployCAN = new CANSparkMax(flywheelIntakeDeployMotorId, MotorType.kBrushless);
    private CANSparkMax rollerCAN = new CANSparkMax(flywheelIntakeRollerMotorId, MotorType.kBrushless);

    private RelativeEncoder deployEncoder = deployCAN.getEncoder();

    private SparkPIDController deployPIDController = deployCAN.getPIDController();

    private double targetRotations = 0;

    public FlywheelIntake () {

        rollerCAN.setInverted(flywheelIntakeRollerMotorInverted);
        deployCAN.setInverted(flywheelIntakeDeployMotorInverted);

        deployCAN.setSoftLimit(SoftLimitDirection.kForward, 0.02f);
        deployCAN.enableSoftLimit(SoftLimitDirection.kForward, true);

        deployPIDController.setP(flywheelIntakeDeployKp);   //Hehe, thats funny, it spells PID.... WAIT
        deployPIDController.setI(flywheelIntakeDeployKi);
        deployPIDController.setD(flywheelIntakeDeployKd);
        //see https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

        deployEncoder.setPositionConversionFactor(flywheelIntakeDeployGearing);
    }

    
    /**
     * set the target angle for the intake to deploy to, in rotations
     */
    public void setDeploymentAngle (double angle) {
        if(angle >= flywheelIntakeDeployMinAngle && angle <= flywheelIntakeDeployMaxAngle)
        {
            return;   //insert error code... code here
        }
        targetRotations = angle; 
        
        deployPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);

    //use the flywheelIntakeDeployGearing constant for gearing to convert to motor rotations
    }

    /**
     * set the speed of the roller, from -1 to 1
     */
    public void setRollerSpeed (double speed) {
        rollerCAN.set(speed);
    }

    /**
     * get the current angle of the intake, in rotations
     */
    public double getDeploymentAngle(){
        return deployEncoder.getPosition();
    }
    /**
     * get whether the deploy motor is at its setpoint
     */

    public boolean deployedToSetpoint () {
        return (targetRotations >= (getDeploymentAngle() - flywheelIntakeDeployTolerance) &&
         targetRotations <= (getDeploymentAngle() + flywheelIntakeDeployTolerance));
        
    }
}