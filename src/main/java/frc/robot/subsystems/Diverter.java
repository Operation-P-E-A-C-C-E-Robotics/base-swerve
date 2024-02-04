package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.Diverter.*;

public class Diverter {
    private final TalonFX deployMotor = new TalonFX(diverterDeployMotorId);
    private final TalonFX rollerMotor = new TalonFX(diverterRollerMotorId);

    private final MotionMagicExpoVoltage deployControl = new MotionMagicExpoVoltage(0);

    private Diverter () {
        deployMotor.getConfigurator().apply(diverterDeployConfigs);
        rollerMotor.setInverted(false);
    }

    public void setDiverterExtension (double position) {
        deployMotor.setControl(deployControl.withPosition(position));
    }

    public void setDiverterRoller (double speed) {
        rollerMotor.set(speed);
    }

    public double getDiverterExtension () {
        return deployMotor.getPosition().getValue();
    }
    
    public boolean atSetpoint () {
        return deployMotor.getClosedLoopError().getValue() < diverterDeployTolerance;
    }

    private static final Diverter instance = new Diverter();
    public static Diverter getInstance(){
        return instance;
    }
}
