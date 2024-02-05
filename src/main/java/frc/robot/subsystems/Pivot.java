package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Util;
import frc.robot.Constants;

import static frc.robot.Constants.Pivot.*;

public class Pivot {
    private final TalonFX pivotMaster = new TalonFX(pivotMasterID);
    private final TalonFX pivotSlave = new TalonFX(pivotSlaveID);

    private final MotionMagicExpoVoltage pivotControl = new MotionMagicExpoVoltage(restingAngle);
    private final ArmFeedforward gravityFeedforward = new ArmFeedforward(0, gravityFeedforwardkG, 0); 

    private final Mechanism2d pivotMech = new Mechanism2d(100, 100);
    private final MechanismLigament2d pivotLigament = pivotMech.getRoot("pivot", 0, 50).append(new MechanismLigament2d("pivot", 0, 50));


    private Pivot () {
        pivotMaster.getConfigurator().apply(pivotConfigs);
        pivotMaster.setInverted(false);

        pivotSlave.setControl(new Follower(pivotMasterID, false));

        SmartDashboard.putData("pivot mech", pivotMech);
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

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(2).withReduction(pivotGearRatio), 
        pivotGearRatio, 
        0.618, 
        Units.inchesToMeters(12), 
        pivotMinAngle, 
        pivotMaxAngle, 
        true, 
        0
    );


    private final TalonFXSimState pivotMasterSim = pivotMaster.getSimState();

    public void simulationPeriodic() {
        pivotSim.setInput(pivotMasterSim.getMotorVoltage());
        pivotSim.update(Constants.period);

        pivotMasterSim.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()));
        pivotMasterSim.setRotorVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec()));

        pivotLigament.setAngle(new Rotation2d(pivotSim.getAngleRads()));

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(pivotMasterSim.getSupplyCurrent()));
    }

    private static final Pivot instance = new Pivot();
    public static Pivot getInstance(){
        return instance;
    }
}
