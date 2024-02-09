package frc.lib.util;

import java.util.HashMap;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Reporter {
    //report a ctre status to the driver station if it is not OK
    public static void report (StatusCode status, String message) {
        if (!status.isOK()) {
            DriverStation.reportError(message + ": " + status.getDescription(), false);
        }
    }

    public static void report (REVLibError status, String message) {
        if (status != REVLibError.kOk) {
            DriverStation.reportError(message + ": " + status, false);
        }
    }

    private static HashMap<String, StringLogEntry> logEntries = new HashMap<>();

    public static void log (StatusCode status, String message) {
        if (!status.isOK()) {
            if(logEntries.containsKey(message)){
                logEntries.get(message).append(status.getDescription());
            } else {
                logEntries.put(message, new StringLogEntry(DataLogManager.getLog(), "Reporter/CTRE errors/" + message));
            }
        }
    }

    public static void log (REVLibError status, String message) {
        if (status != REVLibError.kOk) {
            if(logEntries.containsKey(message)){
                logEntries.get(message).append(status.toString());
            } else {
                logEntries.put(message, new StringLogEntry(DataLogManager.getLog(), "Reporter/REV errors/" + message));
            }
        }
    }
}
