package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber {
    private TalonFX leftMotor = new TalonFX(climberLeftMotorId);
    private TalonFX rightMotor = new TalonFX(climberRightMotorId);
    //declare two TalonFXs, one for the left climber, one for the right climber

    private TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
    private TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();

    private double climberSetpoint = 0.0d;

    private double leftMotorClimberPos = 0.0d;
    private double rightMotorClimberPos = 0.0d;

    public Climber () {
        leftMotor.setInverted(climberLeftMotorIsInverted);
        rightMotor.setInverted(climberRightMotorIsInverted);
        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);
        
        //see https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html
        //use MotionMagicExpo.
    }

    /**
     * sets the position of both sides of the climber, in meters
     * @param position
     */
    public void setClimberPosition (double position) {
        setClimberPosition(position, position);
    }

    /**
     * sets the position of the left and right sides of the climber, in meters
     * @param left the position of the left side of the climber
     * @param right the position of the right side of the climber
     */
    public void setClimberPosition (double left, double right) {

        //use the climberGearing constant for gearing to convert to motor rotations
        leftMotor.setPosition(left);                                       //ask sean  if this is ok :|
        rightMotor.setPosition(right);
    }

    //i changed this from a double to void so i could grab both values and store them in different variables.
    public double getClimberPosition(){
        return 0; //leftMotorClimberPos = leftMotor.getPosition().getValue();   //unable to finish this in time due to statemachine tomfoolery
        //rightMotorClimberPos = rightMotor.getPosition().getValue(); 
    }

    /**
     * get whether the climber is at its setpoint
     */
    public boolean atSetpoint () {
        //unfinished btw
        getClimberPosition();                              //below line checks BOTH climber motors
        return (leftMotorClimberPos >= (climberSetpoint - climberTolerance) && leftMotorClimberPos <= (climberSetpoint + climberTolerance)
                && rightMotorClimberPos >= (climberSetpoint - climberTolerance) && rightMotorClimberPos <= (climberSetpoint + climberTolerance));   //this seems stupid long, lmk if this is too much
    }
}
