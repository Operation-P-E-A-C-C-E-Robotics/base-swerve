package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

public class Climber {
    //declare two TalonFXs, one for the left climber, one for the right climber

    public Climber () {
        //set inversion of the motors so that positive input makes the climber go up

        //configure your climber talons with the MotionMagic gains that are in Constants.Climber
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
    }

    /**
     * get the position of the climber (average of the left and right sides), in meters
     */
    public double getClimberPosition() {
        return 0;
    }

    /**
     * get whether the climber is at its setpoint
     */
    public boolean atSetpoint () {
        //use the climberTolerance constant
        return false;
    }
}
