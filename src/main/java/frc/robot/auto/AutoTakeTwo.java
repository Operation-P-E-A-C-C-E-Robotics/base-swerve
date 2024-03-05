package frc.robot.auto;

import frc.robot.RobotStatemachine;
import frc.robot.RobotStatemachine.SuperstructureState;
import frc.robot.planners.NoteTracker;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoTakeTwo {

    public static final TimedAuto testAuto = new TimedAuto(
        new Action(SuperstructureState.AUTO_AIM, 0, 0.75),
        new Action(SuperstructureState.SHOOT, 0.75, 1, () -> Shooter.getInstance().shotDetected()),
        new Action(SuperstructureState.INTAKE_BACK, 1, 3),
        new Action(SuperstructureState.INTAKE_BACK, Path.DRIVE_OFF_LINE.command, 0.0, 5.0, () -> NoteTracker.getLocation() == NoteTracker.NoteLocation.INDEXING),
        new Action(SuperstructureState.REST, 5, 5.5),
        new Action(SuperstructureState.AUTO_AIM, 5.5, 10, () -> Shooter.getInstance().shotDetected())

    );

    public static class TimedAuto {
        private final Action[] actions;
        private final Timer timer = new Timer();
        private int currentAction = 0;

        public TimedAuto(Action... actions) {
            this.actions = actions;
        }

        public void reset() {
            timer.reset();
            timer.start();
            currentAction = 0;
        }

        public void run(RobotStatemachine superstructure) {
            timer.start();
            for (int i = currentAction; i < actions.length; i++) {
                if (actions[i].wantsRun(timer.get())) {
                    currentAction = i;
                    superstructure.requestSwervePath(actions[i].getPath());
                    superstructure.requestSwerveState(actions[i].getSwerveState());
                    superstructure.requestState(actions[i].getState());
                    return;
                }
            }
        }
    }

    public static class Action {
        private final SuperstructureState state;
        private final SwerveState swerveState;
        private final Command path;
        private final double startTime, endTime;
        private final BooleanSupplier endCondition;

        public Action(SuperstructureState state, SwerveState swerveState, Command path, double startTime, double endTime, BooleanSupplier endCondition) {
            this.state = state;
            this.swerveState = swerveState;
            this.path = path;
            this.startTime = startTime;
            this.endTime = endTime;
            this.endCondition = endCondition;
        }

        public Action(SuperstructureState state, Command path, double startTime, double endTime) {
            this(state, SwerveState.FOLLOW_PATH, path, startTime, endTime, () -> false);
        }

        public Action(RobotStatemachine.SuperstructureState state, Command path, double startTime, double endTime, BooleanSupplier endCondition) {
            this(state, SwerveState.FOLLOW_PATH, path, startTime, endTime, endCondition);
        }

        public Action(SuperstructureState state, SwerveState swerveState, Command path, double startTime, double endTime) {
            this(state, swerveState, path, startTime, endTime, () -> false);
        }

        public Action(SwerveState swerveState, double startTime, double endTime, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, swerveState, new InstantCommand(), startTime, endTime, endCondition);
        }

        public Action(SwerveState swerveState, double startTime, double endTime) {
            this(swerveState, startTime, endTime, () -> false);
        }

        public Action(Command path, double startTime, double endTime, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, SwerveState.LOCK_IN, path, startTime, endTime, endCondition);
        }

        public Action(Command path, double startTime, double endTime) {
            this(path, startTime, endTime, () -> false);
        }

        public Action(SuperstructureState state, double startTime, double endTime, BooleanSupplier endCondition) {
            this(
                state, 
                state == SuperstructureState.AUTO_AIM || state == SuperstructureState.SHOOT ? SwerveState.AIM : SwerveState.LOCK_IN, 
                new InstantCommand(), startTime, endTime, endCondition
            );
        }

        public Action(SuperstructureState state, double startTime, double endTime) {
            this(state, startTime, endTime, () -> false);
        }

        public Action(double startTime, double endTime, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, startTime, endTime, endCondition);
        }

        public Action(double startTime, double endTime) {
            this(startTime, endTime, () -> false);
        }

        public boolean wantsRun(double time) {
            return time >= startTime && time <= endTime && !endCondition.getAsBoolean();
        }

        public SuperstructureState getState() {
            return state;
        }

        public SwerveState getSwerveState() {
            return swerveState;
        }

        public Command getPath() {
            return path;
        }

        public boolean isDone(double time) {
            return time >= endTime && endCondition.getAsBoolean();
        }

        public double getStartTime() {
            return startTime;
        }

        public double getEndTime() {
            return endTime;
        }

        public BooleanSupplier getEndCondition() {
            return endCondition;
        }
    }

    public static enum Path {
        TEST_PATH("test path", 3),
        DRIVE_OFF_LINE("drive off line", 1.5);
    
        public final String pathName;
        public final Command command;
    
        private Path(String pathName, double duration) {
            this.pathName = pathName;
            this.command = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName));
        }
    }
}
