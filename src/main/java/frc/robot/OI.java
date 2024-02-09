package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.Shooter;

public class OI {
    private static final Joystick driverJoystick = new Joystick(0);
    private static final Joystick operatorJoystick = new Joystick(1);
    public static class Swerve{
        public static final DoubleSupplier translation = () -> -driverJoystick.getRawAxis(5);
        public static final DoubleSupplier strafe = () -> -driverJoystick.getRawAxis(4);
        public static final DoubleSupplier rotation = () -> -driverJoystick.getRawAxis(0);
        public static final DoubleSupplier heading = () -> (double) -driverJoystick.getPOV();
        public static final BooleanSupplier useHeading = () -> driverJoystick.getPOV() != -1;
        public static final BooleanSupplier isFieldRelative = () -> driverJoystick.getRawAxis(2) < 0.2;
        public static final BooleanSupplier isLockIn = () -> driverJoystick.getRawAxis(3) > 0.2;
        public static final BooleanSupplier isZeroOdometry = () -> driverJoystick.getRawButton(7);
        public static final BooleanSupplier isOpenLoop = () -> true;
        public static final BooleanSupplier isOverrideHeading = () -> false;
    }

    public static class Modes {
        public static final BooleanSupplier wantsSpeakerMode = () -> false;
        public static final BooleanSupplier wantsAmpMode = () -> false;
        public static final BooleanSupplier wantsClimbMode = () -> false;
    }

    public static class Overrides {
        /* MODE OVERRIDES */ //overrides the state requested by the mode
        public static final BooleanSupplier forceAim = () -> false;
        public static final BooleanSupplier forceIntakeFront = () -> false;
        public static final BooleanSupplier forceIntakeBack = () -> false;
       
        /* STATE OVERRIDES */ //overrides the state requested by the teleop state machine
        public static final BooleanSupplier forceHandoff = () -> false;
        public static final BooleanSupplier forceAmp = () -> false;

        /* DIRECT OVERRIDES */ //directly sets the state of the subsystem
        public static final BooleanSupplier forceTrigger = () -> false;
        public static final BooleanSupplier eject = () -> false;
    }

    public static class ManualInputs {
        public static final DoubleSupplier jogTrigger = () -> 0.0;
        public static final DoubleSupplier jogPivot = () -> 0.0;
        public static final DoubleSupplier jogClimber = () -> 0.0;
    }

    private static final double swerveCurrentRumbleThreshold = 40; //Amps
    private static final double swerveCurrentRumbleScalar = 80; //Amps, how much current gives 100% rumble (0.5 on each side)

    public void updateRumble () {
        if(Shooter.getInstance().shotDetected()) {
            driverJoystick.setRumble(RumbleType.kBothRumble, 0.5);
            operatorJoystick.setRumble(RumbleType.kBothRumble, 0.5);
        }

        var driveCurrent = frc.robot.subsystems.Swerve.getInstance().getTotalDriveCurrent();
        if(driveCurrent > swerveCurrentRumbleThreshold) {
            var rumble = (driveCurrent - swerveCurrentRumbleThreshold) / swerveCurrentRumbleScalar;
            //divide based on strafe amount
            var left = rumble * (0.5 - (Swerve.strafe.getAsDouble() / 2));
            var right = rumble * (0.5 + (Swerve.strafe.getAsDouble() / 2));

            driverJoystick.setRumble(RumbleType.kLeftRumble, left);
            driverJoystick.setRumble(RumbleType.kRightRumble, right);
        }
    }
}
