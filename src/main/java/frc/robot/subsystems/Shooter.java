package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Util;
import frc.robot.Constants;

import static frc.robot.Constants.Shooter.*;

public class Shooter {
    private final TalonFX topFlywheelMotor = new TalonFX(upperFlywheelMotorId);
    private final TalonFX bottomFlywheelMotor = new TalonFX(lowerFlywheelMotorId);

    private final LinearSystem<N1, N1, N1> topFlywheelSystem = LinearSystemId.identifyVelocitySystem(flywheelKv, flywheelKa);

    private final KalmanFilter<N1, N1, N1> topFlywheelObserver = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        topFlywheelSystem,
        VecBuilder.fill(flywheelModelStDev),
        VecBuilder.fill(flywheelEncoderStDev),
        Constants.period
    );

    private final LinearQuadraticRegulator<N1, N1, N1> topFlywheelController = new LinearQuadraticRegulator<>(
        topFlywheelSystem,
        VecBuilder.fill(flywheelErrorTolerance),
        VecBuilder.fill(flywheelControlEffort),
        Constants.period
    );

    private final LinearSystemLoop<N1, N1, N1> topFlywheelLoop = new LinearSystemLoop<>(topFlywheelSystem, topFlywheelController, topFlywheelObserver, 12.0, Constants.period);

    private final LinearSystem<N1, N1, N1> bottomFlywheelSystem = LinearSystemId.identifyVelocitySystem(flywheelKv, flywheelKa);

    private final KalmanFilter<N1, N1, N1> bottomFlywheelObserver = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        bottomFlywheelSystem,
        VecBuilder.fill(flywheelModelStDev),
        VecBuilder.fill(flywheelEncoderStDev),
        Constants.period
    );

    private final LinearQuadraticRegulator<N1, N1, N1> bottomFlywheelController = new LinearQuadraticRegulator<>(
        bottomFlywheelSystem,
        VecBuilder.fill(flywheelErrorTolerance),
        VecBuilder.fill(flywheelControlEffort),
        Constants.period
    );

    private final LinearSystemLoop<N1, N1, N1> bottomFlywheelLoop = new LinearSystemLoop<>(bottomFlywheelSystem, bottomFlywheelController, bottomFlywheelObserver, 12.0, Constants.period);
   
    private final VoltageOut topControl = new VoltageOut(0);
    private final VoltageOut bottomControl = new VoltageOut(0);

    private final CANSparkMax triggerMotor = new CANSparkMax(triggerMotorId, MotorType.kBrushless);

    private final DigitalInput flywheelSwitch = new DigitalInput(flywheelSwitchId);
    private final DigitalInput triggerSwitch = new DigitalInput(triggerSwitchId);

    private final Timer shotTimer = new Timer();

    private Shooter () {
        topFlywheelMotor.getConfigurator().apply(flywheelConfigs);
        bottomFlywheelMotor.getConfigurator().apply(flywheelConfigs);

        topFlywheelMotor.setInverted(false);
        bottomFlywheelMotor.setInverted(false);
        triggerMotor.setInverted(false);
    }

    public void setFlywheelVelocity (double velocity) {
        setFlywheelVelocity(velocity, velocity);
    }

    public void setFlywheelVelocity (double top, double bottom) {
        topFlywheelLoop.setNextR(VecBuilder.fill(top));
        topFlywheelLoop.correct(VecBuilder.fill(topFlywheelMotor.getVelocity().getValue()));
        topFlywheelLoop.predict(Constants.period);

        bottomFlywheelLoop.setNextR(VecBuilder.fill(bottom));
        bottomFlywheelLoop.correct(VecBuilder.fill(bottomFlywheelMotor.getVelocity().getValue()));
        bottomFlywheelLoop.predict(Constants.period);

        topControl.withOutput(topFlywheelLoop.getU(0));
        bottomControl.withOutput(bottomFlywheelLoop.getU(0));

        topFlywheelMotor.setControl(topControl);
        bottomFlywheelMotor.setControl(bottomControl);
    }

    public void setFlywheelPercent (double percent) {
        topFlywheelMotor.set(percent);
        bottomFlywheelMotor.set(percent);
        topFlywheelLoop.reset(VecBuilder.fill(topFlywheelMotor.getVelocity().getValue()));
        bottomFlywheelLoop.reset(VecBuilder.fill(bottomFlywheelMotor.getVelocity().getValue()));
    }

    public void setTrigerPercent (double percent) {
        triggerMotor.set(percent);
    }

    public double getFlywheelVelocity () {
        return (topFlywheelMotor.getVelocity().getValue() + bottomFlywheelMotor.getVelocity().getValue()) / 2;
    }

    public double getFlywheelAcceleration () {
        return (topFlywheelMotor.getAcceleration().getValue() + bottomFlywheelMotor.getAcceleration().getValue()) / 2;
    }

    public boolean flywheelAtTargetVelocity () {
        return Util.inRange(topFlywheelLoop.getError(0), flywheelTolerance) && Util.inRange(bottomFlywheelLoop.getError(0), flywheelTolerance);
    }

    public boolean flywheelSwitchTripped () {
        return flywheelSwitch.get();
    }

    public boolean triggerSwitchTripped () {
        return triggerSwitch.get();
    }

    public boolean hasNote () {
        return false;
    }

    public boolean shotDetected() {
        if (getFlywheelVelocity() > shotDetectionMinVelocity && getFlywheelAcceleration() < shotDetectionAccelerationThreshold) {
            shotTimer.start();
        } 
        if (shotTimer.hasElapsed(shotDetectionTimeThreshold)) {
            shotTimer.stop();
            shotTimer.reset();
            return true;
        }
        return false;
    }

    private static final Shooter instance = new Shooter();
    public static Shooter getInstance(){
        return instance;
    }
}
