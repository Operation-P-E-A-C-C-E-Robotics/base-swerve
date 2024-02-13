package frc.lib.util;

import java.util.HashMap;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Reporter {
    /**
     * Reports a CTRE status code to the driver station
     * if it is an error code.
     * @param status the status code to report
     * @param message the message to report with the status code
     */
    public static void report (StatusCode status, String message) {
        if (!status.isOK()) {
            DriverStation.reportError(message + ": " + status.getDescription(), false);
        }
    }

    /**
     * Reports a REVLib status code to the driver station
     * if it is an error code.
     * @param status the status code to report
     * @param message the message to report with the status code
     */
    public static void report (REVLibError status, String message) {
        if (status != REVLibError.kOk) {
            DriverStation.reportError(message + ": " + status, false);
        }
    }

    private static HashMap<String, StringLogEntry> logEntries = new HashMap<>();

    /**
     * Logs a CTRE status code to the data logger
     * if it is an error code.
     * @param status the status code to log
     * @param message the message to log with the status code
     */
    public static void log (StatusCode status, String message) {
        if (!status.isOK()) {
            if(logEntries.containsKey(message)){
                logEntries.get(message).append(status.getDescription());
            } else {
                logEntries.put(message, new StringLogEntry(DataLogManager.getLog(), "Reporter/CTRE errors/" + message));
            }
        }
    }

    /**
     * Logs a REVLib status code to the data logger
     * if it is an error code.
     * @param status the status code to log
     * @param message the message to log with the status code
     */
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
