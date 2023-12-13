package frc.lib.telemetry;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Handles all the telemetry for the swerve drive. Designed to be used with advantagescope.
 */
public class SwerveTelemetry {
    private static final NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");

    private static final StructPublisher<Pose3d> swervePosePublisher = swerveTable.getStructTopic("Robot Pose", Pose3d.struct).publish();
    private static final StructPublisher<Pose3d> pathplannerTargetPosePublisher = swerveTable.getStructTopic("Target Pose", Pose3d.struct).publish();
    private static final StructArrayPublisher <Pose2d> pathplannerTrajectoryPublisher = swerveTable.getStructArrayTopic("Path", Pose2d.struct).publish();
    
    private static final DoubleArrayPublisher swerveDataPublisher = swerveTable.getDoubleArrayTopic("Swerve Data").publish();
    private static final DoublePublisher measuredXVelocity = swerveTable.getDoubleTopic("Measured X Velocity").publish();
    private static final DoublePublisher measuredYVelocity = swerveTable.getDoubleTopic("Measured Y Velocity").publish();
    private static final DoublePublisher measuredAngularVelocity = swerveTable.getDoubleTopic("Measured Angular Velocity").publish();
    
    private static final DoublePublisher odometryPeriod = swerveTable.getDoubleTopic("Odometry Period").publish();



    private static final NetworkTable swerveCommandTable = swerveTable.getSubTable("Command");
    
    private static final DoublePublisher swerveRequestedXVelocity = swerveCommandTable.getDoubleTopic("Requested X Velocity").publish();
    private static final DoublePublisher swerveRequestedYVelocity = swerveCommandTable.getDoubleTopic("Requested Y Velocity").publish();
    
    private static final DoublePublisher swerveRequestedAngularVelocity = swerveCommandTable.getDoubleTopic("Requested Angular Velocity").publish();
    private static final DoublePublisher swerveRequestedAutoHeadingAngle = swerveCommandTable.getDoubleTopic("Requested Auto Heading Angle").publish();
    private static final BooleanPublisher requestFieldCentricPublisher = swerveCommandTable.getBooleanTopic("Request Field Centric").publish();
    private static final BooleanPublisher requestAutoAnglePublisher = swerveCommandTable.getBooleanTopic("Request Auto Angle").publish();
    private static final BooleanPublisher requestOpenLoopPublisher = swerveCommandTable.getBooleanTopic("Request Open Loop").publish();
    private static final BooleanPublisher requestLockInPublisher = swerveCommandTable.getBooleanTopic("Request Lock In").publish();
    private static final BooleanPublisher requestZeroOdometryPublisher = swerveCommandTable.getBooleanTopic("Request Zero Odometry").publish();


    
    private static final Mechanism2d swerve = new Mechanism2d(5, 5);
    private static final MechanismRoot2d frontLeftModule = swerve.getRoot("front left", 1, 1);
    private static final MechanismRoot2d frontRightModule = swerve.getRoot("front right", 1, 4);
    private static final MechanismRoot2d rearLeftModule = swerve.getRoot("rear left", 4, 1);
    private static final MechanismRoot2d rearRightModule = swerve.getRoot("rear right", 4, 4);

    private static final MechanismLigament2d frontLeftLigament = new MechanismLigament2d("front left ligament", 0, 0);
    private static final MechanismLigament2d frontRightLigament = new MechanismLigament2d("front right ligament", 0, 0);
    private static final MechanismLigament2d rearLeftLigament = new MechanismLigament2d("rear left ligament", 0, 0);
    private static final MechanismLigament2d rearRightLigament = new MechanismLigament2d("rear right ligament", 0, 0);

    private static final Field2d field = new Field2d();

    static {
        frontLeftModule.append(frontLeftLigament);
        frontRightModule.append(frontRightLigament);
        rearLeftModule.append(rearLeftLigament);
        rearRightModule.append(rearRightLigament);

        SmartDashboard.putData("Swerve Mech", swerve);
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            var poseArray = poses.toArray(new Pose2d[0]);
            pathplannerTrajectoryPublisher.accept(poseArray);
            field.getObject("Path").setPoses(poseArray);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            pathplannerTargetPosePublisher.accept(new Pose3d(pose));
            field.getObject("Target Pose").setPose(pose);
        });
    }

    public static void updateSwerveState(SwerveDriveState state, ChassisSpeeds measuredSpeeds) {
        swervePosePublisher.accept(new Pose3d(state.Pose));//TODO update once have 3d odometry stuff
        field.setRobotPose(state.Pose);

        frontLeftLigament.setAngle(state.ModuleStates[0].angle.getDegrees());
        frontRightLigament.setAngle(state.ModuleStates[1].angle.getDegrees());
        rearLeftLigament.setAngle(state.ModuleStates[2].angle.getDegrees());
        rearRightLigament.setAngle(state.ModuleStates[3].angle.getDegrees());

        frontLeftLigament.setLength(state.ModuleStates[0].speedMetersPerSecond / Constants.Swerve.pathfollowingMaxVelocity);
        frontRightLigament.setLength(state.ModuleStates[1].speedMetersPerSecond / Constants.Swerve.pathfollowingMaxVelocity);
        rearLeftLigament.setLength(state.ModuleStates[2].speedMetersPerSecond / Constants.Swerve.pathfollowingMaxVelocity);
        rearRightLigament.setLength(state.ModuleStates[3].speedMetersPerSecond / Constants.Swerve.pathfollowingMaxVelocity);

        /*
        format needed for advantagescope:
        [
            rotation_1, velocity_1,
            rotation_2, velocity_2,
            rotation_3, velocity_3,
            rotation_4, velocity_4
        ]
         */
        double[] swerveData = new double[]{
            state.ModuleStates[0].angle.getDegrees(), state.ModuleStates[0].speedMetersPerSecond,
            state.ModuleStates[1].angle.getDegrees(), state.ModuleStates[1].speedMetersPerSecond,
            state.ModuleStates[2].angle.getDegrees(), state.ModuleStates[2].speedMetersPerSecond,
            state.ModuleStates[3].angle.getDegrees(), state.ModuleStates[3].speedMetersPerSecond
        };
        swerveDataPublisher.accept(swerveData);

        measuredXVelocity.accept(measuredSpeeds.vxMetersPerSecond);
        measuredYVelocity.accept(measuredSpeeds.vyMetersPerSecond);
        measuredAngularVelocity.accept(measuredSpeeds.omegaRadiansPerSecond);
        odometryPeriod.accept(state.OdometryPeriod);
    }

    public static void updateSwerveCommand(double requestedXVelocity, 
                                            double requestedYVelocity, 
                                            double requestedAngularVelocity, 
                                            double requestedAutoHeading, 
                                            boolean isAutoHeading, 
                                            boolean isFieldRelative, 
                                            boolean isOpenLoop, 
                                            boolean isLockIn, 
                                            boolean isZeroOdometry) {
        swerveRequestedXVelocity.accept(requestedXVelocity);
        swerveRequestedYVelocity.accept(requestedYVelocity);
        swerveRequestedAngularVelocity.accept(requestedAngularVelocity);
        swerveRequestedAutoHeadingAngle.accept(requestedAutoHeading);
        requestFieldCentricPublisher.accept(isFieldRelative);
        requestAutoAnglePublisher.accept(isAutoHeading);
        requestOpenLoopPublisher.accept(isOpenLoop);
        requestLockInPublisher.accept(isLockIn);
        requestZeroOdometryPublisher.accept(isZeroOdometry);
    }

}
