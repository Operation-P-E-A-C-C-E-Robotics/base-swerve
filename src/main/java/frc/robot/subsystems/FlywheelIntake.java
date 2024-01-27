package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelIntake.*;

//software for the intake that is on the front of the robot
public class FlywheelIntake {
    //Declare two CANSparkMax motorcontrollers, one for deploy, one for the roller
    //Use the CAN IDs from constants

    public FlywheelIntake () {
        //set inversion of the motors so that positive input makes the roller spin inwards and the intake deploy

        //configure your deploy spark max with the pid gains that are in Constants.FlywheelIntake
        //see https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
    }

    /**
     * set the target angle for the intake to deploy to, in rotations
     */
    public void setDeploymentAngle (double angle) {
        //use the flywheelIntakeDeployGearing constant for gearing to convert to motor rotations
    }

    /**
     * set the speed of the roller, from -1 to 1
     */
    public void setRollerSpeed (double speed) {

    }

    /**
     * get the current angle of the intake, in rotations
     */
    public double getDeploymentAngle () {
        return 0;
    }

    /**
     * get whether the deploy motor is at its setpoint
     */
    public boolean deployedToSetpoint () {
        return false;
    }
}
