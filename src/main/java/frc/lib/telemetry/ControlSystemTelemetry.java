package frc.lib.telemetry;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class ControlSystemTelemetry {
    private static final NetworkTable controlSystemTable = NetworkTableInstance.getDefault().getTable("Control System");
    private static final NetworkTable roboRIOTable = controlSystemTable.getSubTable("RoboRIO");
    private static final NetworkTable pneumaticsTable = controlSystemTable.getSubTable("Pneumatics");
    private static final NetworkTable canTable = controlSystemTable.getSubTable("CAN");
    
    //rio
    private static final DoublePublisher loopTimePublisher = roboRIOTable.getDoubleTopic("RIO Loop Time").publish();
    private static final DoublePublisher commandsOverheadPublisher = roboRIOTable.getDoubleTopic("CommandScheduler Run Duration").publish();
    private static final BooleanPublisher brownoutPublisher = roboRIOTable.getBooleanTopic("Brownout").publish();
    private static final DoublePublisher batteryVoltagePublisher = roboRIOTable.getDoubleTopic("Battery Voltage").publish();
    private static final DoublePublisher brownoutVoltagePublisher = roboRIOTable.getDoubleTopic("Brownout Voltage").publish();  
    private static final DoublePublisher inputVoltagePublisher = roboRIOTable.getDoubleTopic("Input Voltage").publish();
    private static final DoublePublisher inputCurrentPublisher = roboRIOTable.getDoubleTopic("Input Current").publish();
    private static final DoublePublisher v3_3FaultsPublisher = roboRIOTable.getDoubleTopic("3.3v Rail Faults").publish();
    private static final DoublePublisher v3_3CurrentPublisher = roboRIOTable.getDoubleTopic("3.3v Rail Current").publish();
    private static final DoublePublisher v5FaultsPublisher = roboRIOTable.getDoubleTopic("5v Rail Faults").publish();
    private static final DoublePublisher v5CurrentPublisher = roboRIOTable.getDoubleTopic("5v Rail Current").publish();
    private static final DoublePublisher v6FaultsPublisher = roboRIOTable.getDoubleTopic("6v Rail Faults").publish();
    private static final DoublePublisher v6CurrentPublisher = roboRIOTable.getDoubleTopic("6v Rail Current").publish();
    private static final DoublePublisher cpuTemp = roboRIOTable.getDoubleTopic("CPU Temperature").publish();
   
    private static final BooleanPublisher outputsEnabled = roboRIOTable.getBooleanTopic("Outputs Enabled").publish();

    //can
    private static final DoublePublisher canBusUtilization = canTable.getDoubleTopic("CAN Bus Utilization").publish();
    private static final DoublePublisher canBusOffCount = canTable.getDoubleTopic("CAN Bus Off Count").publish();
    private static final DoublePublisher canRxErrorCount = canTable.getDoubleTopic("CAN Rx Error Count").publish();
    private static final DoublePublisher canTxErrorCount = canTable.getDoubleTopic("CAN Tx Error Count").publish();
    private static final DoublePublisher canTxFullCount = canTable.getDoubleTopic("CAN Tx Full Count").publish();

    //pneumatics
    private static final BooleanPublisher compressorEnabled = pneumaticsTable.getBooleanTopic("Compressor Enabled").publish();
    private static final DoublePublisher pneumaticPressure = pneumaticsTable.getDoubleTopic("Pneumatic Pressure").publish();
    private static final StringPublisher pneumaticSolenoidStates = pneumaticsTable.getStringTopic("Solenoid States").publish();

    private static Timer time = new Timer();

    public static void update(PneumaticsBase pcm, double schedulerTime) {
        loopTimePublisher.accept(time.get());
        commandsOverheadPublisher.accept(schedulerTime);
        time.reset();
        time.start();

        brownoutPublisher.accept(RobotController.isBrownedOut());
        batteryVoltagePublisher.accept(RobotController.getBatteryVoltage());
        brownoutVoltagePublisher.accept(RobotController.getBrownoutVoltage());
        inputVoltagePublisher.accept(RobotController.getInputVoltage());
        inputCurrentPublisher.accept(RobotController.getInputCurrent());
        v3_3FaultsPublisher.accept(RobotController.getFaultCount3V3());
        v3_3CurrentPublisher.accept(RobotController.getVoltage3V3());
        v5FaultsPublisher.accept(RobotController.getFaultCount5V());
        v5CurrentPublisher.accept(RobotController.getVoltage5V());
        v6FaultsPublisher.accept(RobotController.getFaultCount6V());
        v6CurrentPublisher.accept(RobotController.getVoltage6V());
        cpuTemp.accept(RobotController.getCPUTemp());

        outputsEnabled.accept(RobotController.isSysActive());

        var canStatus = RobotController.getCANStatus();

        canBusUtilization.accept(canStatus.percentBusUtilization);
        canBusOffCount.accept(canStatus.busOffCount);
        canRxErrorCount.accept(canStatus.receiveErrorCount);
        canTxErrorCount.accept(canStatus.transmitErrorCount);
        canTxFullCount.accept(canStatus.txFullCount);


        if(pcm != null){
            compressorEnabled.accept(pcm.getCompressor());
            pneumaticPressure.accept(pcm.getPressure(0));
            pneumaticSolenoidStates.accept(Integer.toBinaryString(pcm.getSolenoids()));
        }

        RobotController.getBatteryVoltage();
    }
}
