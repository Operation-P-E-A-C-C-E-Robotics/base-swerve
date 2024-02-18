package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Reporter;
import frc.lib.util.Util;
import frc.robot.Robot;

import static frc.robot.Constants.Pivot.*;

public class Pivot {
    /* HARDWARE */
    private final TalonFX pivotMaster = new TalonFX(pivotMasterID);
    private final TalonFX pivotFollower = new TalonFX(pivotFollowerID);
    private final CANcoder pivotEncoder = new CANcoder(pivotConfigs.Feedback.FeedbackRemoteSensorID);

    /* CONTROLLERS / CONTROL REQUESTS */
    private final MotionMagicExpoVoltage pivotControl = new MotionMagicExpoVoltage(restingAngle);
    private final ArmFeedforward gravityFeedforward = new ArmFeedforward(0, gravityFeedforwardkG, 0);

    /* TELEMETRY */
    private final Mechanism2d pivotMech = new Mechanism2d(100, 100);
    private final MechanismLigament2d pivotLigament = pivotMech.getRoot("pivot", 50, 50).append(new MechanismLigament2d("pivot", 30, 0));

    /* STATUS SIGNALS */
    private final StatusSignal <Double> positionSignal;
    private final StatusSignal <Double> velocitySignal;

    private Pivot () {
        Reporter.report(
            pivotMaster.getConfigurator().apply(pivotConfigs),
            "Couldn't configure pivot master"
        );

        pivotMaster.setInverted(false);

        Reporter.report(
            pivotFollower.setControl(new Follower(pivotMasterID, false)),
            "Couldn't configure pivot slave"
        );

        //disable all unused status signals to minimize CAN usage
        ParentDevice.optimizeBusUtilizationForAll(pivotMaster, pivotFollower, pivotEncoder);
        positionSignal = pivotEncoder.getAbsolutePosition();
        velocitySignal = pivotEncoder.getVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(100, positionSignal, velocitySignal);

        SmartDashboard.putData("pivot mech", pivotMech);
    }

    /**
     * set the position of the pivot, with 0 being horizontal
     * @param position the position duh
     */
    public void setPivotPosition (Rotation2d position) {
        var gravity = gravityFeedforward.calculate(getPivotPosition().getRadians(), 0);
        pivotControl.withFeedForward(gravity).withPosition(position.getRotations());
        Reporter.log(pivotMaster.setControl(pivotControl), "couldn't set pivot position");

        if(Robot.isSimulation()) {
            pivotEncoder.getSimState().setRawPosition(position.getRotations());
            pivotLigament.setAngle(getPivotPosition());
        }
    }

    /**
     * Get the latency-compensated pivot position
     * @return the pivot position with 0 being fully horizontal
     */
    public Rotation2d getPivotPosition () {
        Reporter.report(positionSignal.getStatus(), "Couldn't read pivot position");
        var compensatedRotations = BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal);
        return Rotation2d.fromRotations(compensatedRotations);
    }

    public void engageBrake (boolean enable) {
        //TODO
    }

    public boolean atSetpoint () {
        return Util.inRange(pivotMaster.getClosedLoopError().getValue(), pivotTolerance);
    }

    private static final Pivot instance = new Pivot();
    public static Pivot getInstance(){
        return instance;
    }
}
