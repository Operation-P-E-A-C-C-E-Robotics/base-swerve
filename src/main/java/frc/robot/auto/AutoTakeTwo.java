package frc.robot.auto;

import frc.robot.RobotContainer;
import frc.robot.RobotStatemachine;
import frc.robot.RobotStatemachine.SuperstructureState;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoTakeTwo {

    public static final TimedAuto twoNoteCenter = new TimedAuto(
        layupShot(),
        intakeAndFollowPath(Path.DRIVE_OFF_LINE_CENTER),
        shoot(),
        end()
    );

    public static final TimedAuto layupOnly = new TimedAuto(
        layupShot(),
        end()
    );

    private static Action[] layupShot(){
        return new Action[]{
            new Action(SuperstructureState.AIM_LAYUP, 0.75),
            new Action(SuperstructureState.SHOOT, 0.25, () -> Shooter.getInstance().shotDetected())
        };
    }

    private static Action[] intakeAndFollowPath(Path path){
        return new Action[]{
            new Action(SuperstructureState.INTAKE_BACK, 0.5),
            new Action(SuperstructureState.INTAKE_BACK, path.command, path.duration + 1, () -> Shooter.getInstance().flywheelSwitchTripped())
        };
    }

    private static Action[] shoot() {
        return new Action[]{
            new Action(SuperstructureState.REST, 0.5), //allow the shooter to index
            new Action(SuperstructureState.AUTO_AIM, 3, RobotContainer.getInstance()::readyToShoot),
            new Action(SuperstructureState.SHOOT, 0.5)
        };
    }

    private static Action[] end(){
        return new Action[]{new Action(15)};
    }

    public static class TimedAuto {
        private final Action[] actions;
        private final Timer timer = new Timer();
        private int currentAction = 0;

        public TimedAuto(Action... actions) {
            this.actions = actions;
        }

        public TimedAuto(Action[]... actions){
            //concatenate the arrays
            int totalLength = 0;
            for (Action[] action : actions) {
                totalLength += action.length;
            }
            this.actions = new Action[totalLength];
            int index = 0;
            for (Action[] action : actions) {
                for (Action action2 : action) {
                    this.actions[index] = action2;
                    index++;
                }
            }
        }

        public void reset() {
            timer.reset();
            timer.start();
            currentAction = 0;
        }

        public void run(RobotStatemachine superstructure) {
            timer.start();
            // for (int i = currentAction; i < actions.length; i++) {
            //     if (actions[i].wantsRun(timer.get())) {
            //         currentAction = i;
            //         superstructure.requestSwervePath(actions[i].getPath());
            //         superstructure.requestSwerveState(actions[i].getSwerveState());
            //         superstructure.requestState(actions[i].getState());
            //         return;
            //     }
            // }
            if(actions[currentAction].isDone(timer.get()) && !(currentAction + 1 >= actions.length)) {
                currentAction++;
                timer.reset();
            }

            var action = actions[currentAction];
            superstructure.requestSwervePath(action.getPath());
            superstructure.requestSwerveState(action.getSwerveState());
            superstructure.requestState(action.getState());
        }
    }

    public static class Action {
        private final SuperstructureState state;
        private final SwerveState swerveState;
        private final Command path;
        private final double timeout;
        private final BooleanSupplier endCondition;

        public Action(SuperstructureState state, SwerveState swerveState, Command path, double timeout, BooleanSupplier endCondition) {
            this.state = state;
            this.swerveState = swerveState;
            this.path = path;
            this.timeout = timeout;
            this.endCondition = endCondition;
        }

        public Action(SuperstructureState state, Command path, double timeout) {
            this(state, SwerveState.FOLLOW_PATH, path, timeout, () -> false);
        }

        public Action(RobotStatemachine.SuperstructureState state, Command path, double timeout, BooleanSupplier endCondition) {
            this(state, SwerveState.FOLLOW_PATH, path, timeout, endCondition);
        }

        public Action(SuperstructureState state, SwerveState swerveState, Command path, double timeout) {
            this(state, swerveState, path, timeout, () -> false);
        }

        public Action(SwerveState swerveState, double timeout, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, swerveState, new InstantCommand(), timeout, endCondition);
        }

        public Action(SwerveState swerveState, double timeout) {
            this(swerveState, timeout, () -> false);
        }

        public Action(Command path, double timeout, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, SwerveState.LOCK_IN, path, timeout, endCondition);
        }

        public Action(Command path, double timeout) {
            this(path, timeout, () -> false);
        }

        public Action(SuperstructureState state, double timeout, BooleanSupplier endCondition) {
            this(
                state, 
                state == SuperstructureState.AUTO_AIM || state == SuperstructureState.SHOOT ? SwerveState.AIM : SwerveState.LOCK_IN, 
                new InstantCommand(), timeout, endCondition
            );
        }

        public Action(SuperstructureState state, double timeout) {
            this(state, timeout, () -> false);
        }

        public Action(double timeout, BooleanSupplier endCondition) {
            this(SuperstructureState.REST, timeout, endCondition);
        }

        public Action(double timeout) {
            this(timeout, () -> false);
        }

        public boolean wantsRun(double time) {
            return time <= timeout && !endCondition.getAsBoolean();
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
            return time >= timeout || endCondition.getAsBoolean();
        }

        public double gettimeout() {
            return timeout;
        }

        public BooleanSupplier getEndCondition() {
            return endCondition;
        }
    }

    public static enum Path {
        TEST_PATH("test path", 3),
        DRIVE_OFF_LINE_CENTER("drive off line center", 2.1),
        DRIVE_OFF_LINE_AMPSIDE("drive off line ampside", 2.0),
        DRIVE_OFF_LINE_STAGESIDE("drive off line stageside", 1.8),
        AMPSIDE_NOTE_TO_CENTER_NOTE("ampside note to center note", 2.6),
        CENTERSIDE_NOTE_TO_STAGESIDE_NOTE("center note to stageside note", 2.0);
    
        public final String pathName;
        public final Command command;
        public final double duration;
    
        private Path(String pathName, double duration) {
            this.pathName = pathName;
            this.duration = duration;
            this.command = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName));
        }
    }
}
