package frc.robot.planners;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Very important planner to keep us from colliding with the stage.
 * Says when we're driving under the stage and hence the shooter needs to stow.
 */
public class StageAvoidancePlanner {
    private Supplier<Pose2d> robotPose;

    //Danger zone is where the shooter should stow (approaching the stage)
    //TODO
    private final double DANGER_TOP_LEFT_X = 0;
    private final double DANGER_TOP_LEFT_Y = 0;
    private final double DANGER_BOTTOM_RIGHT_X = 0;
    private final double DANGER_BOTTOM_RIGHT_Y = 0;

    boolean shouldStow = false;

    public StageAvoidancePlanner (Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;
    }

    public void update() {
        var pose = robotPose.get();

        shouldStow = pose.getX() > DANGER_TOP_LEFT_X && pose.getX() < DANGER_BOTTOM_RIGHT_X
            && pose.getY() > DANGER_TOP_LEFT_Y && pose.getY() < DANGER_BOTTOM_RIGHT_Y;
    }

    /**
     * @return whether the shooter should stow to avoid a collision.
     */
    public boolean shouldStow() {
        return shouldStow;
    }
}
