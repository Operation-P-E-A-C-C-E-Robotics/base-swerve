package frc.robot.subsystems;

import static frc.robot.Constants.TriggerIntake.*;

public class TriggerIntake {
    public TriggerIntake () {

    }

    /**
     * set the target angle for the intake to deploy to, in rotations
     */
    public void setDeploymentAngle (double angle) {

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
