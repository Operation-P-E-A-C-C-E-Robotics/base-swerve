package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.util.Util;

import static frc.robot.Constants.Pivot.*;

public class Pivot {
    private final TalonFX pivotMaster = new TalonFX(pivotMasterID);
    private final TalonFX pivotSlave = new TalonFX(pivotSlaveID);

    private final MotionMagicExpoVoltage pivotControl = new MotionMagicExpoVoltage(restingAngle);
    private final ArmFeedforward gravityFeedforward = new ArmFeedforward(0, gravityFeedforwardkG, 0); 

    private Pivot () {
        pivotMaster.getConfigurator().apply(pivotConfigs);
        pivotMaster.setInverted(false);

        pivotSlave.setControl(new Follower(pivotMasterID, false));
    }

    public void setPivotPosition (double position) {
        var gravity = gravityFeedforward.calculate(getPivotPosition(), 0);
        pivotMaster.setControl(pivotControl.withFeedForward(gravity).withPosition(position));
    }

    public double getPivotPosition () {
        return pivotMaster.getPosition().getValue();
    }

    public boolean atSetpoint () {
        return Util.inRange(pivotMaster.getClosedLoopError().getValue(), pivotTolerance);
    }

    private static final Pivot instance = new Pivot();
    public static Pivot getInstance(){
        return instance;
    }
}
