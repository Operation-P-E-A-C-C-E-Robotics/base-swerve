package frc.robot.auto;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TeleopStatemachine;
import frc.robot.TeleopStatemachine.TeleopState;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Shooter;

public class Autonomous {

    public static final AutoMode testAuto = new AutoMode(
        new AutoSegment(
            Path.TEST_PATH,
            new TimedRobotState(TeleopState.INTAKE_FRONT, 0, 0.5),
            new TimedRobotState(TeleopState.SHOOT, 0.5, 5, () -> Shooter.getInstance().shotDetected())
        )
    );

    public static class TimedRobotState {
        public final TeleopState teleopState;
        public final double startPercent;
        public final double endPercent;
        public final BooleanSupplier endCondition;

        public TimedRobotState(TeleopState teleopState, double startPercent, double endPercent, BooleanSupplier endCondition) {
            this.teleopState = teleopState;
            this.startPercent = startPercent;
            this.endPercent = endPercent;
            this.endCondition = endCondition;
        }

        public TimedRobotState(TeleopState teleopState, double startPercent, double endPercent) {
            this(teleopState, startPercent, endPercent, () -> false);
        }
        
        public boolean wantsRun(double time) {
            return time >= startPercent && time <= endPercent && !endCondition.getAsBoolean();
        }
    }

    public static class AutoSegment {
        private final Path path;
        private final TimedRobotState[] timedStates;

        public AutoSegment(Path path, TimedRobotState... timedStates) {
            this.path = path;
            this.timedStates = timedStates;
        }

        public TeleopState getTeleopState(double time) {
            for (TimedRobotState state : timedStates) {
                if (state.wantsRun(time / path.duration)) {
                    return state.teleopState;
                }
            }
            return TeleopState.REST;
        }

        public Path getPath() {
            return path;
        }

        public boolean isDone(double time) {
            if(time > path.duration) return true;
            for (TimedRobotState state : timedStates) {
                if (state.wantsRun(time / path.duration)) {
                    return false;
                }
            }
            return true;
        }
    }

    public static class AutoMode {
        private final AutoSegment[] segments;
        private int currentSegment = 0;
        private boolean newSegment = true;

        public AutoMode(AutoSegment... segments) {
            this.segments = segments;
        }

        public void run(SwerveStatemachine swerve, TeleopStatemachine robot) {
            AutoSegment current = segments[currentSegment];

            if(newSegment) {
                robot.requestSwerveState(SwerveState.FOLLOW_PATH);
                swerve.setPathCommand(current.getPath().command);
                newSegment = false;
            }

            double time = swerve.getPathTime();
            robot.requestState(current.getTeleopState(time));

            if(current.isDone(time)) {
                currentSegment++;
                newSegment = true;
            }
        }
    }

    public static enum Path {
        TEST_PATH("test path", 1);
    
        public final String pathName;
        public final double duration;
        public final Command command;
    
        private Path(String pathName, double duration) {
            this.pathName = pathName;
            this.duration = duration;
            this.command = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName));
        }
    }
}
