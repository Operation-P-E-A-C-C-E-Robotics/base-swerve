package frc.lib.telemetry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Robot;

/**
 * Telemetry for a limelight camera, intended for use with apriltag tracking.
 * Doesn't need to log too much, since the limelight's own networktables are
 * already being logged. Just reformats some data for advantagescope.
 * <ul>
 *    <li>Array of corners, formatted for advantagescope</li>
 *    <li>Primary tag pose3d from robot</li>
 *    <li>Primary tag pose3d from field</li>
 *    <li>Raw botpose as pose3d</li>
 * </ul>
 * 
 * A HOT MESS COURTESY OF PEACCY
 * I LEGINTIMATELY HAVE NO IDEA WHAT I'M DOING
 */
public class LimelightTelemetry {
    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("Limelight");
    private static final StructArrayPublisher<Pose3d> tagPosePublisher = limelightTable.getStructArrayTopic("Tag Poses From Robot", Pose3d.struct).publish();
    private static final StructArrayPublisher<Pose3d> tagPoseFromFieldPublisher = limelightTable.getStructArrayTopic("Tag Poses From Field", Pose3d.struct).publish();
    private static final StructPublisher<Pose3d> rawBotPosePublisher = limelightTable.getStructTopic("Raw BotPose", Pose3d.struct).publish();

    public static void update (String llName, Pose3d robotPose){
        var results = LimelightHelpers.getLatestResults(llName).targetingResults;
        Pose3d[] tagPosesFromField = new Pose3d[results.targets_Fiducials.length];
        Pose3d[] tagPosesFromRobot = new Pose3d[results.targets_Fiducials.length];
        for(int i = 0; i < results.targets_Fiducials.length; i++){
            tagPosesFromField[i] = results.targets_Fiducials[i].getTargetPose_RobotSpace();
            tagPosesFromRobot[i] = robotPose.plus(new Transform3d(tagPosesFromField[i].getTranslation(), tagPosesFromField[i].getRotation()));
        }
        
        tagPosePublisher.accept(tagPosesFromRobot);
        tagPoseFromFieldPublisher.accept(tagPosesFromField);
        if(results.botpose.length == 6){
            rawBotPosePublisher.accept(robotPose);
        }

        if(Robot.isSimulation()){
            tagPoseFromFieldPublisher.accept(new Pose3d[] {
                new Pose3d(0, 0, 1, new Rotation3d(0, 0, 0)),
                new Pose3d(2, 2, 1, new Rotation3d(0, 0, 0)),
                new Pose3d(2, 0, 0.4, new Rotation3d(1, 0, 0)),
            });

            rawBotPosePublisher.accept(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        }
    }
}
