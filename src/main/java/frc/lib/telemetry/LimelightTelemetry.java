package frc.lib.telemetry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.lib.safety.Value;
import frc.lib.vision.Limelight;

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
    private static final DoubleArrayPublisher cornersXPublisher = limelightTable.getDoubleArrayTopic("Corners X").publish();
    private static final DoubleArrayPublisher cornersYPublisher = limelightTable.getDoubleArrayTopic("Corners Y").publish();
    private static final StructPublisher<Pose3d> primaryTagPosePublisher = limelightTable.getStructTopic("Primary Tag From Robot", Pose3d.struct).publish();
    private static final StructPublisher<Pose3d> primaryTagPoseFromFieldPublisher = limelightTable.getStructTopic("Primary Tag From Field", Pose3d.struct).publish();
    private static final StructPublisher<Pose3d> rawBotPosePublisher = limelightTable.getStructTopic("Raw BotPose", Pose3d.struct).publish();

    public static final void update(Limelight limelight, Pose3d robotPose){
        Value<double[]> tcornxy = limelight.getCorners();
        if(tcornxy.isNormal()) {
            double[] corners = tcornxy.get(new double[]{});
            
            //make sure length is even and nonzero:
            if(corners.length % 2 == 0 && corners.length != 0) {
                double[] cornersX = new double[corners.length / 2];
                double[] cornersY = new double[corners.length / 2];
                for (int i = 0; i < corners.length; i++) {
                    if (i % 2 == 0) {
                        cornersX[i / 2] = corners[i];
                    } else {
                        cornersY[i / 2] = corners[i];
                    }
                }
                cornersXPublisher.accept(cornersX);
                cornersYPublisher.accept(cornersY);
            }

            Value<double[]> primaryTagPoseRobot = limelight.getTagFromRobot();
            if (primaryTagPoseRobot.isNormal()){
                double[] primaryTagPoseRobotArray = primaryTagPoseRobot.get(new double[]{});
                Pose3d primaryTagPoseFromRobot = new Pose3d(
                    primaryTagPoseRobotArray[0], 
                    primaryTagPoseRobotArray[1], 
                    primaryTagPoseRobotArray[2], 
                    new Rotation3d(
                        primaryTagPoseRobotArray[3], 
                        primaryTagPoseRobotArray[4], 
                        primaryTagPoseRobotArray[5]
                    )
                );

                primaryTagPosePublisher.accept(primaryTagPoseFromRobot);

                Pose3d primaryTagPoseFromField = robotPose.plus(new Transform3d(new Pose3d(), primaryTagPoseFromRobot));
                primaryTagPoseFromFieldPublisher.accept(primaryTagPoseFromField);
            }

            Value<Pose3d> rawBotPose = limelight.getBotpose();
            if (rawBotPose.isNormal()){
                rawBotPosePublisher.accept(rawBotPose.get(new Pose3d()));
            }
        }
    }
}
