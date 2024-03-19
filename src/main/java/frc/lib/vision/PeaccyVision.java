package frc.lib.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Util;
import frc.lib.vision.ApriltagCamera.*;

/**
 * PeaccyVision is a class that manages multiple ApriltagCameras and their results.
 * (an ApriltagCamera is an interface that can be implemented with any camera that can detect apriltags, e.g. Limelight or Photonvision)
 * it is responsible for combining the results of multiple cameras and providing a single source of truth for the robot's position and orientation.
 * it also uses a very made-up algorithm to determine the best standard deviation to use when
 * feeding the results to a pose estimator.
 */
public class PeaccyVision {
    private static final double INITIALIZE_ERROR = 500;
    private static final double DISTANCE_DRIVEN_ERROR_WEIGHT = 1;
    private static final double TAG_ERROR_REDUCTION = 0.5;
    private static final double ACCELERATION_PENALTY = 100;
    private static final double ACCELERATION_PENALTY_THRESHOLD = 0.5;
    private static final double VISION_DISTANCE_FROM_CURRENT_ERROR_WEIGHT = 0.5;

    private static final double MIN_STDEV = 0.1;
    private static final double MAX_STDEV = 5.0;
    private static final double STDEV_ERROR_WEIGHT = 0.1;
    
    private static final double STDEV_YAW_MULTIPLIER = 15;


    private ApriltagCamera[] cameras;

    private double odometryError = INITIALIZE_ERROR;

    private Pose2d visionPose = new Pose2d();
    private Pose2d prevOdometryPose = new Pose2d();
    private double stDev = MAX_STDEV;
    private double timestamp = Timer.getFPGATimestamp();

    private boolean hasUpdated = false;

    /**
     * Create a new PeaccyVision object with the given cameras.
     * intelligently combines the results of multiple cameras to provide a single source of truth for the robot's position and orientation.
     * and uses a very made-up algorithm to determine the best standard deviation to use when feeding the results to a pose estimator.
     * @param cameras the cameras IN ORDER OF TRUST. the first camera is the most trusted, the last camera is the least trusted.
     */
    public PeaccyVision(ApriltagCamera... cameras){
        this.cameras = cameras;
    }

    public void update(Pose2d odometryPose, double acceleration) {
        var visionResult = getMeasurement(odometryPose);
        var deltaDistance = odometryPose.getTranslation().getDistance(prevOdometryPose.getTranslation());
        prevOdometryPose = odometryPose;
        if(visionResult.isEmpty()) {
            hasUpdated = false;
            return;
        }

        this.visionPose = visionResult.get().pose;
        this.timestamp = visionResult.get().timestamp;
        var numTags = visionResult.get().numTags;

        var visionDiscrepancy = visionPose.getTranslation().getDistance(odometryPose.getTranslation());
        var accelerationPenalty = acceleration > ACCELERATION_PENALTY_THRESHOLD ? ACCELERATION_PENALTY : 0;

        odometryError += deltaDistance * DISTANCE_DRIVEN_ERROR_WEIGHT;
        odometryError += accelerationPenalty;
        odometryError += visionDiscrepancy * VISION_DISTANCE_FROM_CURRENT_ERROR_WEIGHT;
        odometryError *= TAG_ERROR_REDUCTION * numTags;

        stDev = odometryError * STDEV_ERROR_WEIGHT;
        stDev = Util.limit(acceleration, MIN_STDEV, MAX_STDEV);

        hasUpdated = true;
    }

    public Pose2d getPose(){
        return visionPose;
    }

    public Matrix<N3, N1> getStDev(){
        return VecBuilder.fill(stDev, stDev, stDev * STDEV_YAW_MULTIPLIER);
    }

    public double getTimestamp(){
        return timestamp;
    }

    public boolean hasUpdated(){
        return hasUpdated;
    }

    private Optional<ApriltagPoseMeasurement> getMeasurement(Pose2d odoPose) {
        var results = new VisionResults[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            results[i] = cameras[i].getLatestResults(odoPose);
        }
        
        //calculate weighted average pose based on camera trust.
        //use the timestamp of the most trusted camera with a target as the timestamp of the vision pose.
        var totalWeight = 0.0;
        var timestamp = Timer.getFPGATimestamp();
        var hasSetTimestamp = false;
        
        for(int i = 0; i < results.length; i++){
            if(results[i].hasUpdated() && results[i].getNumTags() > 0){
                totalWeight += cameras[i].getTrust();
                if(!hasSetTimestamp){
                    timestamp = results[i].getTimestamp();
                    hasSetTimestamp = true;
                }
            }
        }
        
        if(totalWeight == 0) return Optional.empty();
        
        var numTags = 0;
        var weightedPose = new Pose2d();
        for(int i = 0; i < results.length; i++){
            if(!results[i].hasUpdated() || results[i].getNumTags() == 0) continue;
            weightedPose = new Pose2d(
                weightedPose.getX() + results[i].getPose2d().getX() * cameras[i].getTrust(),
                weightedPose.getY() + results[i].getPose2d().getY() * cameras[i].getTrust(),
                new Rotation2d(weightedPose.getRotation().getRadians() + results[i].getPose2d().getRotation().getRadians() * cameras[i].getTrust())
            );
            numTags += results[i].getNumTags();
        }

        weightedPose = new Pose2d(
            weightedPose.getX() / totalWeight,
            weightedPose.getY() / totalWeight,
            new Rotation2d(weightedPose.getRotation().getRadians() / totalWeight)
        );

        return Optional.of(new ApriltagPoseMeasurement(visionPose, timestamp, numTags));
    }

    private static class ApriltagPoseMeasurement{
        public final Pose2d pose;
        public final double timestamp;
        public final int numTags;

        public ApriltagPoseMeasurement(Pose2d pose, double timestamp, int numTags){
            this.pose = pose;
            this.timestamp = timestamp;
            this.numTags = numTags;
        }
    }
}
