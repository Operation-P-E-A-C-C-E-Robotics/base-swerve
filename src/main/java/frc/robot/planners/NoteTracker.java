package frc.robot.planners;

import frc.robot.TeleopStatemachine.TeleopState;
import frc.robot.subsystems.Shooter;

/**
 * Make sure we know where our notes are at all times.
 * Looks at the state of the robot (whether the shooter sees a note,
 * whether the shooter has detected a shot, whether the flipper is running, etc.)
 */
public class NoteTracker {
    private static NoteLocation location = NoteLocation.NONE;

    public static void update (TeleopState state) {
        if(Shooter.getInstance().flywheelSwitchTripped() || Shooter.getInstance().triggerSwitchTripped()) {
            location = NoteLocation.INDEXING;
        } else {
            if (location == NoteLocation.INDEXING) location = NoteLocation.SHOOTER;
        }
        
        if(Shooter.getInstance().shotDetected()) {
            location = NoteLocation.NONE;
        }

        if(state == TeleopState.HANDOFF) {
            location = NoteLocation.FLIPPER;
        }

        if(state == TeleopState.PLACE_AMP || state == TeleopState.PLACE_TRAP) {
            location = NoteLocation.NONE;
        }

    }

    public static NoteLocation getLocation () {
        return location;
    }
    
    public enum NoteLocation {
        NONE, INDEXING, SHOOTER, FLIPPER,
    }
}
