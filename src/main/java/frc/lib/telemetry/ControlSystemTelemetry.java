package frc.lib.telemetry;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.RobotController;

public class ControlSystemTelemetry {
    private static final NetworkTable controlSystemTable = NetworkTableInstance.getDefault().getTable("Control System");
    
    private static final DoublePublisher loopTimePublisher = controlSystemTable.getDoubleTopic("RIO Loop Time").publish();

    private static final BooleanPublisher compressorEnabled = controlSystemTable.getBooleanTopic("Compressor Enabled").publish();
    private static final DoublePublisher pneumaticPressure = controlSystemTable.getDoubleTopic("Pneumatic Pressure").publish();
    private static final StringPublisher pneumaticSolenoidStates = controlSystemTable.getStringTopic("Solenoid States").publish();

    private static double time = RobotController.getFPGATime();

    public static void update(PneumaticsBase pcm) {
        loopTimePublisher.accept(RobotController.getFPGATime() - time);
        time = RobotController.getFPGATime();

        if(pcm != null){
            compressorEnabled.accept(pcm.getCompressor());
            pneumaticPressure.accept(pcm.getPressure(0));
            pneumaticSolenoidStates.accept(Integer.toBinaryString(pcm.getSolenoids()));
        }
    }
}
