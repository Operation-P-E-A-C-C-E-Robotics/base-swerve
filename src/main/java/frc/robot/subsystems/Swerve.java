package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.Inspiration;
import frc.lib.swerve.PeaccefulSwerve;
import frc.lib.swerve.SwerveDescription;
import frc.lib.swerve.SwerveDescription.PidGains;
import frc.lib.telemetry.LimelightTelemetry;
import frc.lib.telemetry.SwerveTelemetry;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.*;

public class Swerve extends SubsystemBase {
    protected final PeaccefulSwerve swerve;

    private final SwerveRequest.ApplyChassisSpeeds autonomousRequest = new SwerveRequest.ApplyChassisSpeeds()
                                                                                        .withDriveRequestType(DriveRequestType.Velocity);
    private final SendableChooser<Pose2d> poseSeedChooser = new SendableChooser<>();

    // private LimelightHelper limelight;

    public Swerve() {
        swerve = SwerveDescription.generateDrivetrain(
            dimensions, 
            frontLeftIDs, 
            frontRighIDs, 
            rearLeftIDs, 
            rearRightIDs, 
            gearing, 
            offsets, 
            inversion, 
            physics, 
            driveGains, 
            angleGains, 
            pigeonCANId, 
            invertSteerMotors
        );

        swerve.setSteerCurrentLimit(steerMotorCurrentLimit);

        //pathplanner config
        AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, this::getChassisSpeeds, this::drive, pathFollowerConfig, () -> false, this);

        //log swerve state data as fast as it comes in
        swerve.registerTelemetry((SwerveDriveState state) -> {
            SwerveTelemetry.updateSwerveState(state, ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation()), swerve.getPose3d());
        });

        poseSeedChooser.setDefaultOption("zero", new Pose2d());
        poseSeedChooser.addOption("test", new Pose2d(1, 1, new Rotation2d()));
        SmartDashboard.putData("POSE SEED", poseSeedChooser);
        SmartDashboard.putBoolean("seed pose", false);

        System.out.println("DriveTrain Initialized");
    }

    /**
     * make it go.
     * @param request the request to apply to the drivetrain.
     */
    public void drive(SwerveRequest request) {
        swerve.setControl(request);
    }

    /**
     * make it go in auto.
     * @param speeds the chassis speeds to apply to the drivetrain.
     */
    public void drive(ChassisSpeeds speeds) {
        drive(autonomousRequest.withSpeeds(speeds));
    }

    /**
     * the missile knows where it is at all times. it knows this because it knows where it isn't.
     * @return the pose of the robot.
     */
    public Pose2d getPose () {
        if(swerve.odometryIsValid()) return swerve.getState().Pose;
        return new Pose2d();
    }

    /**
     * this missile even knows how fast it's traveling. it knows this because it knows how fast it isn't traveling.
     * @return the chassis speeds of the robot.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerve.getChassisSpeeds();
    }
    
    /**
     * sometimes, the missile forgets where it is, and it's not even where it's been.
     */
    public void resetOdometry() {
        swerve.seedFieldRelative();
    }

    /**
     * sometimes, we need to tell the missile where it is, and it's not even where it's been.
     * By subtracting where it's been from where it is, or where it's going from where it was, we get
     * where it should be.
     * @param pose the pose to set the robot to.
     */
    public void resetOdometry(Pose2d pose) {
        swerve.seedFieldRelative(pose);
    }

    /**
     * drive in a straight line to a target pose. THIS IS DUMB BECAUSE
     * THE RETURNED COMMAND ALWAYS FOLLOWS THE SAME PATH AND DOESN'T REGENERATE.
     * @param target the goal pose
     * @return the command to follow the path
     */
    // public Command driveToPose(Pose2d target){
    //     //the target rotation is the angle of the curve, and we want to go in a straight line, so it
    //     //needs to be the angle between the robot and the target
    //     Rotation2d targetRotation = target.minus(getPose()).getRotation();

    //     //bezier curve from the current pose to the target pose
    //     List<Translation2d> waypoints = PathPlannerPath.bezierFromPoses(
    //         getPose(), 
    //         new Pose2d(target.getTranslation(), targetRotation)
    //     );

    //     PathPlannerPath path = new PathPlannerPath(
    //         waypoints,
    //         Constants.Swerve.autoMaxSpeed,
    //         new GoalEndState(0.0, target.getRotation())
    //     );

    //     return AutoBuilder.followPathWithEvents(path);
    // }

    public double getTotalDriveCurrent(){
        return swerve.getTotalDriveCurrent();
    }

    public void updateDriveGains(PidGains gains){
        swerve.applyDriveConfigs(gains);
    }

    public void updateAngleGains(PidGains gains){
        swerve.applySteerConfigs(gains);
    }

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("seed pose", false)) {
            // swerve.tareEverything();
            resetOdometry(poseSeedChooser.getSelected());
            SmartDashboard.putBoolean("seed pose", false);
        }

        //update odometry from limelight
        var results = LimelightHelpers.getLatestResults(Constants.Swerve.primaryLLName).targetingResults;
        if(results.botpose.length == 6) {
            Pose2d pose = results.getBotPose2d_wpiBlue();
            swerve.addVisionMeasurement(pose, results.timestamp_RIOFPGA_capture); //todo right timestamp?
        }

        LimelightTelemetry.update(primaryLLName, swerve.getPose3d());
    }

    @Override
    public void simulationPeriodic() {
        swerve.updateSimState(Constants.period, 12);
    }

    public void register(Joystick j){
        Inspiration.fullPeacce(j);
    }
}

