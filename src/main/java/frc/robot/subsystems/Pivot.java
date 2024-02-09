package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Util;
import frc.robot.Robot;

import static frc.robot.Constants.Pivot.*;

public class Pivot {
    private final TalonFX pivotMaster = new TalonFX(pivotMasterID);
    private final TalonFX pivotSlave = new TalonFX(pivotSlaveID);
    private final CANcoder pivotEncoder = new CANcoder(pivotConfigs.Feedback.FeedbackRemoteSensorID);

    private final MotionMagicExpoVoltage pivotControl = new MotionMagicExpoVoltage(restingAngle);
    private final ArmFeedforward gravityFeedforward = new ArmFeedforward(0, gravityFeedforwardkG, 0); 

    private final Mechanism2d pivotMech = new Mechanism2d(100, 100);
    private final MechanismLigament2d pivotLigament = pivotMech.getRoot("pivot", 50, 50).append(new MechanismLigament2d("pivot", 30, 0));


    private Pivot () {
        pivotMaster.getConfigurator().apply(pivotConfigs);
        pivotMaster.setInverted(false);

        pivotSlave.setControl(new Follower(pivotMasterID, false));

        SmartDashboard.putData("pivot mech", pivotMech);
    }

    public void setPivotPosition (Rotation2d position) {
        var gravity = gravityFeedforward.calculate(getPivotPosition().getRadians(), 0);
        pivotMaster.setControl(pivotControl.withFeedForward(gravity).withPosition(position.getRotations()));
        if(Robot.isSimulation()) {
            pivotEncoder.getSimState().setRawPosition(position.getRotations());
            pivotLigament.setAngle(getPivotPosition());
        }
    }

    public Rotation2d getPivotPosition () {
        return Rotation2d.fromRotations(pivotMaster.getPosition().getValue());
    }

    public boolean atSetpoint () {
        return Util.inRange(pivotMaster.getClosedLoopError().getValue(), pivotTolerance);
    }

    private static final Pivot instance = new Pivot();
    public static Pivot getInstance(){
        return instance;
    }
}
