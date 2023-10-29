package frc.lib.sensors;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.safety.Value;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class LimelightHelper {
    private static final double FOCAL_LENGTH = (1*83)/0.32;//(1 * 240) / 0.32; //183 px = 0.21 meters
    NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    NetworkTable limelight;

    private DoubleArraySubscriber botpose, campose, camtran, tcornxy, tc;
    private DoubleSubscriber tv, tx, ty, ta, ts, tl, tshort, tlong, thor, tvert;
    private IntegerSubscriber getpipe, tid;
    private IntegerPublisher ledMode, camMode, stream, snapshot, pipeline;
    private DoubleArrayPublisher crop;
    private MedianFilter xFilter = new MedianFilter(10);

    private DoubleSubscriber dSub(String name){
        return limelight.getDoubleTopic(name).subscribe(0);
    }

    private DoubleArraySubscriber daSub(String name){
        return limelight.getDoubleArrayTopic(name).subscribe(new double[0]);
    }

    private IntegerSubscriber iSub(String name){
        return limelight.getIntegerTopic(name).subscribe(0);
    }

    private IntegerPublisher iPub(String name){
        return limelight.getIntegerTopic(name).publish();
    }

    private DoubleArrayPublisher daPub(String name){
        return limelight.getDoubleArrayTopic(name).publish();
    }


    /**
     * TODO only initialize variables that are used, and cache values to avoid unnecessary resource usage
     * a nice limelight helper for color object and
     * apriltags
     * @param networktablesName the limelight's table in networktables
     */
    public LimelightHelper(String networktablesName){
        limelight = networkTables.getTable(networktablesName);
        botpose = daSub("botpose");
        campose = daSub("campose");
    }

    //NETWORKTABLES API:
    public boolean hasTarget(){
        if(tv == null) tv = dSub("tv");
        return tv.get() == 1;
    }

    public Value<Double> getTargetX(){
        if(!hasTarget()) return Value.notAvailable();
        if(tx == null) tx = dSub("tx");
        return Value.of(tx.get());
    }

    public Value<Double> getTargetY(){
        if(!hasTarget()) return Value.notAvailable();
        if(ty == null) ty = dSub("ty");
        return Value.of(ty.get());
    }

    public Value<Double> getTargetArea(){
        if(!hasTarget()) return Value.notAvailable();
        if(ta == null) ta = dSub("ta");
        return Value.of(ta.get());
    }

    public Value<Double> getTargetSkew(){
        if(!hasTarget()) return Value.notAvailable();
        if(ts == null) ts = dSub("ts");
        return Value.of(ts.get());
    }

    public Value<Double> getFittedShortSideLength(){
        if(!hasTarget()) return Value.notAvailable();
        if(tshort == null) tshort = dSub("tshort");
        return Value.of(tshort.get());
    }

    public Value<Double> getFittedLongSideLength(){
        if(!hasTarget()) return Value.notAvailable();
        if(tlong == null) tlong = dSub("tlong");
        return Value.of(tlong.get());
    }

    public Value<Double> getRoughWidth(){
        if(!hasTarget()) return Value.notAvailable();
        if(thor == null) thor = dSub("thor");
        return Value.of(thor.get());
    }

    public Value<Double> getRoughHeight(){
        if(!hasTarget()) return Value.notAvailable();
        if(tvert == null) tvert = dSub("tvert");
        return Value.of(tvert.get());
    }

    public double getPipeline(){
        if(getpipe == null) getpipe = iSub("getpipe");
        return getpipe.get();
    }

    public double getLatency(){
        if(tl == null) tl = dSub("tl");
        return tl.get() / 1000;
    }

    public Value<double[]> getCameraTranslation(){
        if(camtran == null) camtran = daSub("camtran");
        var val = camtran.get();
        if(val.length != 6) return Value.notAvailable();
        return new Value<>(val);
    }

    public double getApriltagID(){
        if(tid == null) tid = iSub("tid");
        return tid.get();
    }

    public Value<double[]> getCorners(){
        if(tcornxy == null) tcornxy = daSub("tcornxy");
        var val = tcornxy.get();
        if(val.length == 0) return Value.notAvailable();
        return new Value<>(val);
    }

    public double[] getCrosshairColor(){
        if(tc == null) tc = daSub("tc");
        return tc.get();
    }

    public void setLedMode(LEDMode mode){
        if(ledMode == null) ledMode = iPub("ledMode");
        ledMode.set(mode.ordinal());
    }

    public void setCamMode(CamMode mode){
        if(camMode == null) camMode = iPub("camMode");
        camMode.set(mode.ordinal());
    }

    public void setSecondaryCameraMode(StreamMode mode) {
        if(stream == null) stream = iPub("stream");
        stream.set(mode.ordinal());
    }

    public void setCrop(double x1, double x2,  double y1, double y2){
        if(crop == null) crop = daPub("crop");
        double[] cropArray = {x1, x2, y1, y2};
        crop.set(cropArray);
    }

    public void setPipeline(int pipeline){
        if(this.pipeline == null) this.pipeline = iPub("pipeline");
        this.pipeline.set(pipeline);
    }

    public Value<Double> getFilteredX(){
        var targetX = getTargetX();
        if(targetX.getState() != Value.ValueState.NORMAL) {
            xFilter.reset();
            return Value.notAvailable();
        }
        return Value.of(xFilter.calculate(targetX.get(0.0)));
    }

    //GAMEPIECES:

    /**
     * get the distance to an irregular object (e.g. cone or cube)
     * uses the closer approximation of both the width and the height.
     * @param objectHeight height of object (meters)
     * @param objectWidth width of object (meters)
     * @return approximate distance
     */
    public Value<Double> getDistance(double objectHeight, double objectWidth){
        var roughHeight = getRoughHeight();
        var roughWidth = getRoughWidth();
        if(roughHeight.getState() != Value.ValueState.NORMAL ||
                roughWidth.getState() != Value.ValueState.NORMAL) return Value.notAvailable();
        return Value.of((objectHeight * FOCAL_LENGTH) / getRoughHeight().get(0.0));
    }

    /**
     * get the equivalent of the yellow box in limelight software
     */
    public Value<BoundingBox> getBoundingBox(){
        var corners = getCorners().get(new double[0]);
        if(corners.length < 2) return Value.notAvailable();
        double minX = corners[0],
                minY = corners[1],
                maxX = corners[0],
                maxY = corners[1];

        for(int i = 2; i < corners.length; i+=2){
            var x = corners[i];
            var y = corners[i+1];
            if(x < minX) minX = x;
            if(x > maxX) maxX = x;
            if(y < minY) minY = y;
            if(y > maxY) maxY = y;
        }

        return new Value<>(new BoundingBox(minX, minY, maxX, maxY));
    }


    //APRILTAGS:

    double halfFieldWidth = 16.48/2;
    double halfFieldHeight = 8.1/2;

    int divergentVisionReadings = 0;
    static final double ERRONIOUS_VISION_THRESHOLD = 1; //meters
    static final double RESET_VISION_THRESHOLD = 10; //divergent readings

    /**
     * update a pose estimator from vision measurements
     * @param estimator the pose estimator to update
     */
    public void updatePoseEstimator(DifferentialDrivePoseEstimator estimator, DoubleSupplier drivetrainLeft, DoubleSupplier drivetrainRight, Supplier<Rotation2d> imuHeading){
        var visionMeasurements = botpose.readQueue();
        var currentPose = estimator.getEstimatedPosition();

        //add vision measurements to the estimator
        for(TimestampedDoubleArray i : visionMeasurements){
            var time = Timer.getFPGATimestamp() - getLatency() - 0.011;
            double[] val = i.value;
            if(val.length == 6) {
                var visionPose = new Pose2d(
                        val[0] + halfFieldWidth,
                        val[1] + halfFieldHeight,
                        Rotation2d.fromDegrees(val[5])
                );
                //if the vision measurement is too far off, add to the divergent vision readings and don't add it to the estimator
                if(visionPose.getTranslation().getDistance(currentPose.getTranslation()) > ERRONIOUS_VISION_THRESHOLD){
                    divergentVisionReadings++;
                } else {
                    estimator.addVisionMeasurement(visionPose, time);
                    divergentVisionReadings--;
                }
                //if there are too many divergent vision readings, reset the estimator to the vision pose
                if(divergentVisionReadings > RESET_VISION_THRESHOLD){
                    divergentVisionReadings = 0;
                    estimator.resetPosition(imuHeading.get(), drivetrainLeft.getAsDouble(), drivetrainRight.getAsDouble(), visionPose);
                }
            }
        }
    }

    /**
     * get the pose of the robot from apriltags
     * @return the pose of the robot
     */
    public Value<Pose3d> getBotpose(){
        var pose = botpose.get();
        if(pose.length < 6) return Value.notAvailable();
        return new Value<>(new Pose3d(
                pose[0],
                pose[1],
                pose[2],
                new Rotation3d(
                        pose[3],
                        pose[4],
                        pose[5]
                )
        ));
    }

    /**
     * get the pose of the camera from apriltags
     */
    public Pose2d getCameraPose() {
        var camData = campose.get();
        if (camData.length == 3) {
            return new Pose2d(camData[0], camData[1], new Rotation2d());
        }
        return new Pose2d();
    }

    public static class BoundingBox{
        double x1, y1, x2, y2;
        public BoundingBox(double x1, double y1, double x2, double y2){
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
        }
        public Translation2d getCenter(){
            return new Translation2d((x1 + x2)/2, (y1 + y2) / 2);
        }
        public double getHeight(){
            return Math.abs(x2 - x1);
        }
        public double getWidth(){
            return Math.abs(y2 - y1);
        }
        public double getArea(){
            return getWidth()*getHeight();
        }
    }

    public enum LEDMode{
        PIPELINE,
        OFF,
        BLINK,
        ON
    }
    public enum CamMode{
        VISION,
        DRIVER
    }
    public enum StreamMode{
        SIDE_BY_SIDE,
        PIP_MAIN,
        PIP_SECONDARY
    }
}
