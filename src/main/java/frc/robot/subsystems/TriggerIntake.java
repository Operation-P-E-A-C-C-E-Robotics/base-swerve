package frc.robot.subsystems;

import static frc.robot.Constants.TriggerIntake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.util.Reporter;
import frc.lib.util.Util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class TriggerIntake {
    /* HARDWARE */
    private CANSparkMax deployMotor = new CANSparkMax(triggerIntakeDeployMotorId, MotorType.kBrushless);
    private CANSparkMax rollerMotor = new CANSparkMax(triggerIntakeRollerMotorId, MotorType.kBrushless);

    private RelativeEncoder deployEncoder = deployMotor.getEncoder();

    /* CONTROLLERS */
    private SparkPIDController deployController = deployMotor.getPIDController();

    /* STATE */
    private double targetRotation = 0.0;

    /* TELEMETRY */
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Intake");
    private final DoublePublisher deployAnglePublisher = table.getDoubleTopic("Rear Deploy Angle").publish();

    DataLog log = DataLogManager.getLog();
    DoubleLogEntry targetDeployAngleLog = new DoubleLogEntry(log, "Intake/Target Rear Deploy Angle");
    BooleanLogEntry deployedToSetpointLog = new BooleanLogEntry(log, "Intake/Rear Deployed to Setpoint");
    DoubleLogEntry rollerSpeedLog = new DoubleLogEntry(log, "Intake/Rear Roller Speed");

    private TriggerIntake () {
        // configure motors n whatnot
        deployMotor.restoreFactoryDefaults();

        rollerMotor.setInverted(triggerIntakeRollerMotorInverted);
        deployMotor.setInverted(triggerIntakeDeployMotorInverted);
        
        Reporter.report(
            deployMotor.setSoftLimit(SoftLimitDirection.kForward, triggerIntakeDeployMaxAngle), 
            "Couldn't set rear intake forward soft limit");
        Reporter.report(
            deployMotor.setSoftLimit(SoftLimitDirection.kReverse, triggerIntakeDeployMinAngle), 
            "Couldn't set rear intake reverse soft limit"
        );
        Reporter.report(
            deployMotor.enableSoftLimit(SoftLimitDirection.kForward, false), 
            "Couldn't enable rear intake forward soft limit"
        );
        Reporter.report(
            deployMotor.enableSoftLimit(SoftLimitDirection.kReverse, false), 
            "Couldn't enable rear intake reverse soft limit"
        );

        Reporter.report(
            deployController.setP(triggerIntakeDeployKp), 
            "Couldn't configure rear intake kP"
        );
        Reporter.report(
            deployController.setI(triggerIntakeDeployKi), 
            "Couldn't configure rear intake kI"
        );
        Reporter.report(
            deployController.setD(triggerIntakeDeployKd), 
            "Couldn't configure rear intake kD"
        );

        Reporter.report(
            deployEncoder.setPositionConversionFactor(triggerIntakeDeployGearing),
            "Couldn't configure rear intake encoder gearing"
        );


        Reporter.report(
            deployMotor.setSmartCurrentLimit(triggerIntakeDeployStallCurrentLimit, triggerIntakeDeployFreeCurrentLimit),
            "Couldn't configure rear intake current limits"
        );

        deployMotor.burnFlash();
    }

    /**
     * Set the deployment angle of the trigger intake.
     * an ange of 0 is the stowed position, and higher angles are more deployed.
     * @param angle the angle to set the trigger intake to
     */
    public void setDeploymentAngle (Rotation2d angle) {
        targetRotation = angle.getRotations(); 
        Reporter.report(
            deployController.setReference(angle.getRotations(), CANSparkMax.ControlType.kPosition),
            "couldn't set rear intake deploy angle"
        );
    }

    /**
     * Set the speed of the roller on the trigger intake.
     * @param speed the speed of the roller (-1 to 1, positive intakes)
     */
    public void setRollerSpeed (double speed) {
        rollerMotor.set(speed);
    }

    /**
     * Get the current deployment angle of the trigger intake.
     * @return the current deployment angle
     */
    public Rotation2d getDeploymentAngle () {
        return Rotation2d.fromRotations(deployEncoder.getPosition());
    }

    /**
     * @return whether the trigger intake has attained is goal deployment angle.
     */
    public boolean deployedToSetpoint () {
        return Util.inRange(getDeploymentAngle().getRotations() - targetRotation, triggerIntakeDeployTolerance);
    }


    /**
     * Write telemetry to the data logger and network tables
     */
    public void writeTelemetry() {
        deployAnglePublisher.accept(getDeploymentAngle().getDegrees());
        targetDeployAngleLog.append(targetRotation);
        deployedToSetpointLog.append(deployedToSetpoint());
        rollerSpeedLog.append(rollerMotor.get());
    }

    private static final TriggerIntake instance = new TriggerIntake();
    public static TriggerIntake getInstance(){
        return instance;
    }
}
