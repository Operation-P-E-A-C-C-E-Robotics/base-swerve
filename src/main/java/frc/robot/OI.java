package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    private static final Joystick driverJoystick = new Joystick(0);
    public static class DriveTrain{
        public static final DoubleSupplier translation = () -> -driverJoystick.getRawAxis(5);
        public static final DoubleSupplier strafe = () -> -driverJoystick.getRawAxis(4);
        public static final DoubleSupplier rotation = () -> -driverJoystick.getRawAxis(0);
        public static final DoubleSupplier heading = () -> (double) -driverJoystick.getPOV();
        public static final BooleanSupplier useHeading = () -> driverJoystick.getPOV() != -1;
        public static final BooleanSupplier isFieldRelative = () -> driverJoystick.getRawAxis(2) < 0.2;
        public static final BooleanSupplier isLockIn = () -> driverJoystick.getRawAxis(3) > 0.2;
        public static final BooleanSupplier isZeroOdometry = () -> driverJoystick.getRawButton(7);
        public static final BooleanSupplier isOpenLoop = () -> false;
    }
}
