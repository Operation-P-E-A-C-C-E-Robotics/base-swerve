package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.Inspiration;
import frc.lib.swerve.PeaccefulSwerve;
import frc.lib.swerve.SwerveDescription;
import frc.lib.swerve.SwerveDescription.PidGains;
import frc.lib.telemetry.SwerveTelemetry;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.vision.ApriltagCamera;
import frc.lib.vision.PeaccyVision;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.*;

public class Swerve extends SubsystemBase {
    protected final PeaccefulSwerve swerve;

    private final SwerveRequest.ApplyChassisSpeeds autonomousRequest = new SwerveRequest.ApplyChassisSpeeds()
                                                                                        .withDriveRequestType(DriveRequestType.Velocity);
    private final SendableChooser<Pose2d> poseSeedChooser = new SendableChooser<>();

    // private LimelightHelper limelight;

    private static PeaccyVision eyes = new PeaccyVision(
        new ApriltagCamera.ApriltagLimelight(Constants.Swerve.primaryLLName, 0.1)
    );

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
        var pose = swerve.getState().Pose;
        if (pose == null) return new Pose2d();
        return pose;
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
     * a workaround for CTRE's moronic zeroing behavior.
     */
    public void zeroFieldCentric() {
        var cachedPose = getPose();
        resetOdometry(AllianceFlipUtil.apply(new Pose2d()));
        resetOdometry();
        resetOdometry(cachedPose);
    }
    
    public PeaccyVision getCameras(){
        return eyes;
    }

    public Rotation3d getGyroAngle() {
        return swerve.getRotation3d();
    }

    public double getTotalDriveCurrent(){
        return swerve.getTotalDriveCurrent();
    }

    /**
     * DO NOT use this in normal operation. For calibration only
     * @param gains the new gains to apply to the drive motors.
     */
    public void updateDriveGains(PidGains gains){
        swerve.applyDriveConfigs(gains);
    }

    /**
     * DO NOT use this in normal operation. For calibration only
     * @param gains the new gains to apply to the steer motors
     */
    public void updateAngleGains(PidGains gains){
        swerve.applySteerConfigs(gains);
    }

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("seed pose", false)) {
            var startPose = poseSeedChooser.getSelected();
            resetOdometry(new Pose2d(
                AllianceFlipUtil.apply(startPose.getTranslation()),
                startPose.getRotation()
            ));;
            SmartDashboard.putBoolean("seed pose", false);
        }

        BaseStatusSignal.refreshAll(swerve.getPigeon2().getAccelerationX(), swerve.getPigeon2().getAccelerationY(), swerve.getPigeon2().getAccelerationZ());
        var acceleration = swerve.getPigeon2().getAccelerationX().getValue() + swerve.getPigeon2().getAccelerationY().getValue() + swerve.getPigeon2().getAccelerationZ().getValue();
        eyes.update(getPose(), acceleration, new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond).getNorm());
        if(eyes.hasUpdated()){
            swerve.addVisionMeasurement(
                eyes.getPose(),
                eyes.getTimestamp(),
                eyes.getStDev()
            );
        }
    }

    @Override
    public void simulationPeriodic() {
        swerve.updateSimState(Constants.period, 12);
    }

    public void register(Joystick j){
        Inspiration.fullPeacce(j);
    }
}

