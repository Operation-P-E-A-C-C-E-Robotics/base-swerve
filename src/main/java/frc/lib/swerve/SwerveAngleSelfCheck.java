package frc.lib.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.motion.Trajectory;

public class SwerveAngleSelfCheck extends Command {
    private PeaccefulSwerve swerve;
    private NetworkTable swerveTestNT = NetworkTableInstance.getDefault().getTable("Test").getSubTable("Swerve Angle Self Check");

    //how closely all the modules are moving together
    private DoublePublisher module0CohesionPub = swerveTestNT.getDoubleTopic("Module 0 Cohesion").publish();
    private DoublePublisher module1CohesionPub = swerveTestNT.getDoubleTopic("Module 1 Cohesion").publish();
    private DoublePublisher module2CohesionPub = swerveTestNT.getDoubleTopic("Module 2 Cohesion").publish();
    private DoublePublisher module3CohesionPub = swerveTestNT.getDoubleTopic("Module 3 Cohesion").publish();

    //accumulated current draw
    private DoublePublisher module0AccumCurrentPub = swerveTestNT.getDoubleTopic("Module 0 Accum Current").publish();
    private DoublePublisher module1AccumCurrentPub = swerveTestNT.getDoubleTopic("Module 1 Accum Current").publish();
    private DoublePublisher module2AccumCurrentPub = swerveTestNT.getDoubleTopic("Module 2 Accum Current").publish();
    private DoublePublisher module3AccumCurrentPub = swerveTestNT.getDoubleTopic("Module 3 Accum Current").publish();

    //peak current draw
    private DoublePublisher module0PeakCurrentPub = swerveTestNT.getDoubleTopic("Module 0 Peak Current").publish();
    private DoublePublisher module1PeakCurrentPub = swerveTestNT.getDoubleTopic("Module 1 Peak Current").publish();
    private DoublePublisher module2PeakCurrentPub = swerveTestNT.getDoubleTopic("Module 2 Peak Current").publish();
    private DoublePublisher module3PeakCurrentPub = swerveTestNT.getDoubleTopic("Module 3 Peak Current").publish();

    //how long it takes to get within 1 degree of the target angle
    private DoublePublisher module0DurationPub = swerveTestNT.getDoubleTopic("Module 0 Angle Duration").publish();
    private DoublePublisher module1DurationPub = swerveTestNT.getDoubleTopic("Module 1 Angle Duration").publish();
    private DoublePublisher module2DurationPub = swerveTestNT.getDoubleTopic("Module 2 Angle Duration").publish();
    private DoublePublisher module3DurationPub = swerveTestNT.getDoubleTopic("Module 3 Angle Duration").publish();

    //accumulated error of the angle controller
    private DoublePublisher module0AngleErrorPub = swerveTestNT.getDoubleTopic("Module 0 Angle Error").publish();
    private DoublePublisher module1AngleErrorPub = swerveTestNT.getDoubleTopic("Module 1 Angle Error").publish();
    private DoublePublisher module2AngleErrorPub = swerveTestNT.getDoubleTopic("Module 2 Angle Error").publish();
    private DoublePublisher module3AngleErrorPub = swerveTestNT.getDoubleTopic("Module 3 Angle Error").publish();

    //how fast the module moves while spinning
    private DoublePublisher module0RotationalRatePub = swerveTestNT.getDoubleTopic("Module 0 Rotational Rate").publish();
    private DoublePublisher module1RotationalRatePub = swerveTestNT.getDoubleTopic("Module 1 Rotational Rate").publish();
    private DoublePublisher module2RotationalRatePub = swerveTestNT.getDoubleTopic("Module 2 Rotational Rate").publish();
    private DoublePublisher module3RotationalRatePub = swerveTestNT.getDoubleTopic("Module 3 Rotational Rate").publish();

    //accumulated linear position controller error
    private DoublePublisher module0DriveErrorPub = swerveTestNT.getDoubleTopic("Module 0 Drive Error").publish();
    private DoublePublisher module1DriveErrorPub = swerveTestNT.getDoubleTopic("Module 1 Drive Error").publish();
    private DoublePublisher module2DriveErrorPub = swerveTestNT.getDoubleTopic("Module 2 Drive Error").publish();
    private DoublePublisher module3DriveErrorPub = swerveTestNT.getDoubleTopic("Module 3 Drive Error").publish();

    private DoublePublisher module0ScorePub = swerveTestNT.getDoubleTopic("Module 0 Score").publish();
    private DoublePublisher module1ScorePub = swerveTestNT.getDoubleTopic("Module 1 Score").publish();
    private DoublePublisher module2ScorePub = swerveTestNT.getDoubleTopic("Module 2 Score").publish();
    private DoublePublisher module3ScorePub = swerveTestNT.getDoubleTopic("Module 3 Score").publish();
    private DoublePublisher overallScorePub = swerveTestNT.getDoubleTopic("Overall Score").publish();

    private BooleanPublisher module0PassedPub = swerveTestNT.getBooleanTopic("Module 0 Passed").publish();
    private BooleanPublisher module1PassedPub = swerveTestNT.getBooleanTopic("Module 1 Passed").publish();
    private BooleanPublisher module2PassedPub = swerveTestNT.getBooleanTopic("Module 2 Passed").publish();
    private BooleanPublisher module3PassedPub = swerveTestNT.getBooleanTopic("Module 3 Passed").publish();
    private BooleanPublisher overallPassedPub = swerveTestNT.getBooleanTopic("Overall Passed").publish();

    private DoublePublisher driveOdometryErrorPub = swerveTestNT.getDoubleTopic("Drive Odometry Error").publish();
    private double driveOdometryError = 0;

    private final ModuleTestData[] moduleTestData = new ModuleTestData[4];

    //Ranges of acceptable values. beyond these ranges, the test will fail
    //TODO arbitrary selections
    private final double MAX_PEAK_CURRENT = 40; //amps, for angle motors
    private final double MAX_ACCUM_CURRENT = 100; //amps, for angle motors
    private final double MAX_COHESION = 1; //sum of differences between module angles in rotations
    private final double MAX_FINAL_ERROR_ANGLE = 50; //rotations
    private final double MAX_DURATION_90 = 0.5; //seconds
    private final double MAX_ACCUM_ERROR_ANGLE = 400; //arbitrary
    private final double MIN_ROTATIONAL_RATE = 1; //rotations per second (probably)
    private final double MAX_ACCUM_ERROR_DRIVE = 100; //arbitrary
    private final double MAX_ACCUM_CURRENT_DRIVE = 400;

    //Weights of each parameter in module scores. Higher score = worse. 0 is "perfect".
    //TODO arbitrary selections
    private final double PEAK_CURRENT_WEIGHT = 1; //multiple of the max current draw of the modules
    private final double ACCUM_CURRENT_WEIGHT = 1; //multiple of the accumulated current draw of the modules
    private final double COHESION_WEIGHT = 1; //multiple of the difference between the module angles
    private final double FINAL_ANGLE_ERROR_WEIGHT = 10000; //multiple of the FINAL angle error of the modules when going to an angle
    private final double DURATION_90_WEIGHT = 100; //multiple of the time it takes to get to a 90 degree angle
    private final double ACCUM_ERROR_ANGLE_WEIGHT = 20; //multiple of the accumulated error of the angle controller

    private final SwerveRequest.PointWheelsAt angleTestRequest = new SwerveRequest.PointWheelsAt();
    private Timer angle90Timer = new Timer();
    private Timer spinTimer = new Timer();
    private Timer angleForwardTimer = new Timer();

    private Stage stage = Stage.ANGLE_FORWARD;

    /**
     * Swerve Self Tester! :D
     * First, it will orient the wheels forward, then at 90 degrees, then spin them, then drive 1 meter forward
     * It checks for module current draw, how similar the module's motion is ("cohesion"), and how well the module tracks it's angle.
     * @param swerve
     * @param requirements
     */
    public SwerveAngleSelfCheck(PeaccefulSwerve swerve, double maxVelocity, double maxAcceleration, Subsystem... requirements){
        this.swerve = swerve;

        addRequirements(requirements);
    }

    @Override
    public void initialize(){
        stage = Stage.ANGLE_FORWARD;
        for(int i = 0; i < moduleTestData.length; i++){
            moduleTestData[i] = new ModuleTestData();
        }
        angle90Timer.stop();
        angle90Timer.reset();

        spinTimer.stop();
        spinTimer.reset();

        angleForwardTimer.reset();
        angleForwardTimer.start();
        moduleAttained90Test = new boolean[]{false, false, false, false};
    }
    boolean[] moduleAttained90Test = new boolean[]{false, false, false, false};

    @Override
    public void execute(){
        ModuleIterData[] moduleIterData = new ModuleIterData[4];
        for(int i = 0; i < moduleIterData.length; i++){
            moduleIterData[i] = ModuleIterData.getModuleData(i, swerve, stage == Stage.ANGLE_90 ? 0.25 : 0);
        }
        switch(stage){
            case ANGLE_FORWARD:
                swerve.setControl(angleTestRequest.withModuleDirection(new Rotation2d()));

                if(angleForwardTimer.get() > 0.5) {
                    //check to make sure the modules reach the setpoint
                    for(int i = 0; i < moduleTestData.length; i++){
                        var delta = moduleIterData[i].angle; //should be 0 when pointing forward
                        if(delta > MAX_FINAL_ERROR_ANGLE){
                            moduleTestData[i].passed = false;
                        }
                        moduleTestData[i].score += FINAL_ANGLE_ERROR_WEIGHT * delta;
                    }
                    stage = Stage.ANGLE_90;
                                    angle90Timer.start();
                }
                break;
            case ANGLE_90:
                swerve.setControl(angleTestRequest.withModuleDirection(Rotation2d.fromDegrees(90)));
                for(int i = 0; i < moduleTestData.length; i++){
                    var delta = moduleIterData[i].angle - 0.25; //should be 0 when pointing 90 degrees (1/4 rotation)
                    SmartDashboard.putNumber("module " + i + " angle", moduleIterData[i].angle);
                    SmartDashboard.putNumber("module " + i + " delta", delta);
                    //did we reach the setpoint
                    if(delta < MAX_FINAL_ERROR_ANGLE){
                        if(!moduleAttained90Test[i]){
                            moduleTestData[i].duration = angle90Timer.get();
                            moduleTestData[i].score += DURATION_90_WEIGHT * angle90Timer.get();
                        }
                        moduleAttained90Test[i] = true;
                    }

                    //if we don't reach the setpoint within the allowed amount of time, fail
                    if(angle90Timer.get() > MAX_DURATION_90 && delta > MAX_FINAL_ERROR_ANGLE && !moduleAttained90Test[i]){
                        moduleTestData[i].duration = angle90Timer.get();
                        moduleTestData[i].passed = false;
                    }

                    //calculate the cohesion
                    for(int j = 0; j < moduleIterData.length; j++){
                        if(i == j) continue;
                        moduleTestData[i].cohesion += Math.abs(moduleIterData[i].angle - moduleIterData[j].angle);
                    }

                    //accumulate current and angle error
                    moduleTestData[i].angleAccumCurrent += moduleIterData[i].angleCurrentDraw;
                    moduleTestData[i].angleError += moduleIterData[i].angleError;
                }

                if(angle90Timer.get() > 0.5) {
                    for(int i = 0; i < moduleTestData.length; i++){
                        var delta = moduleIterData[i].angle - 0.25; //should be 0 when pointing 90 degrees (1/4 rotation)

                        //if we don't reach the setpoint before the button is pressed, fail
                        if(delta > MAX_FINAL_ERROR_ANGLE){
                            moduleTestData[i].duration = angle90Timer.get();

                            //inside the if statement because if we already reached the setpoint, this is already set
                            moduleTestData[i].score += DURATION_90_WEIGHT * angle90Timer.get();
                            moduleTestData[i].passed = false;
                        }

                        //check accumulated current draw
                        if(moduleTestData[i].angleAccumCurrent > MAX_ACCUM_CURRENT){
                            moduleTestData[i].passed = false;
                        }
                        moduleTestData[i].score += moduleTestData[i].angleAccumCurrent * ACCUM_CURRENT_WEIGHT;

                        //check accumulated angle error
                        if(moduleTestData[i].angleError > MAX_ACCUM_ERROR_ANGLE){
                            moduleTestData[i].passed = false;
                        }
                        moduleTestData[i].score += moduleTestData[i].angleError * ACCUM_ERROR_ANGLE_WEIGHT;

                        //check cohesion
                        if(moduleTestData[i].cohesion > MAX_COHESION){
                            moduleTestData[i].passed = false;
                        }
                        moduleTestData[i].score += moduleTestData[i].cohesion * COHESION_WEIGHT;
                    }
                    stage = Stage.DONE;
                    swerve.seedFieldRelative();
                    spinTimer.reset();
                    spinTimer.start();
                }
                break;
            case SPIN_TEST:
                if(spinTimer.get() < 0.5){
                    swerve.spinAngleMotors(1);

                    //record the max rotational rate & current draw:
                    for(int i = 0; i < moduleTestData.length; i++){
                        moduleTestData[i].rotationalRate = Math.max(moduleTestData[i].rotationalRate, moduleIterData[i].rotationalRate);
                        moduleTestData[i].anglePeakCurrent = Math.max(moduleTestData[i].anglePeakCurrent, moduleIterData[i].angleCurrentDraw);
                    }
                } else {
                    swerve.spinAngleMotors(0);
                }

                if(spinTimer.get() > 0.5){
                    for(int i = 0; i < moduleTestData.length; i++){
                        //check the rotational rate
                        if(moduleTestData[i].rotationalRate < MIN_ROTATIONAL_RATE){
                            moduleTestData[i].passed = false;
                        }
                        moduleTestData[i].score += moduleTestData[i].rotationalRate * MIN_ROTATIONAL_RATE;

                        //check the peak current draw
                        if(moduleTestData[i].anglePeakCurrent > MAX_PEAK_CURRENT){
                            moduleTestData[i].passed = false;
                        }
                        moduleTestData[i].score += moduleTestData[i].anglePeakCurrent * PEAK_CURRENT_WEIGHT;
                    }
                    swerve.tareEverything();
                    stage = Stage.DONE;
                }
                break;
            case DONE:
                break;
            default:
                break;
        }
        writeTelemetry();
    }

    @Override
    public boolean isFinished(){
        return stage == Stage.DONE;
    }

    private void writeTelemetry(){
        module0CohesionPub.accept(moduleTestData[0].cohesion);
        module1CohesionPub.accept(moduleTestData[1].cohesion);
        module2CohesionPub.accept(moduleTestData[2].cohesion);
        module3CohesionPub.accept(moduleTestData[3].cohesion);

        module0AccumCurrentPub.accept(moduleTestData[0].angleAccumCurrent);
        module1AccumCurrentPub.accept(moduleTestData[1].angleAccumCurrent);
        module2AccumCurrentPub.accept(moduleTestData[2].angleAccumCurrent);
        module3AccumCurrentPub.accept(moduleTestData[3].angleAccumCurrent);

        module0PeakCurrentPub.accept(moduleTestData[0].anglePeakCurrent);
        module1PeakCurrentPub.accept(moduleTestData[1].anglePeakCurrent);
        module2PeakCurrentPub.accept(moduleTestData[2].anglePeakCurrent);
        module3PeakCurrentPub.accept(moduleTestData[3].anglePeakCurrent);

        module0DurationPub.accept(moduleTestData[0].duration);
        module1DurationPub.accept(moduleTestData[1].duration);
        module2DurationPub.accept(moduleTestData[2].duration);
        module3DurationPub.accept(moduleTestData[3].duration);

        module0AngleErrorPub.accept(moduleTestData[0].angleError);
        module1AngleErrorPub.accept(moduleTestData[1].angleError);
        module2AngleErrorPub.accept(moduleTestData[2].angleError);
        module3AngleErrorPub.accept(moduleTestData[3].angleError);

        module0RotationalRatePub.accept(moduleTestData[0].rotationalRate);
        module1RotationalRatePub.accept(moduleTestData[1].rotationalRate);
        module2RotationalRatePub.accept(moduleTestData[2].rotationalRate);
        module3RotationalRatePub.accept(moduleTestData[3].rotationalRate);

        module0DriveErrorPub.accept(moduleTestData[0].driveError);
        module1DriveErrorPub.accept(moduleTestData[1].driveError);
        module2DriveErrorPub.accept(moduleTestData[2].driveError);
        module3DriveErrorPub.accept(moduleTestData[3].driveError);

        module0ScorePub.accept(moduleTestData[0].score);
        module1ScorePub.accept(moduleTestData[1].score);
        module2ScorePub.accept(moduleTestData[2].score);
        module3ScorePub.accept(moduleTestData[3].score);

        module0PassedPub.accept(moduleTestData[0].passed);
        module1PassedPub.accept(moduleTestData[1].passed);
        module2PassedPub.accept(moduleTestData[2].passed);
        module3PassedPub.accept(moduleTestData[3].passed);

        driveOdometryErrorPub.accept(driveOdometryError);

        overallScorePub.accept(moduleTestData[0].score + moduleTestData[1].score + moduleTestData[2].score + moduleTestData[3].score);
        overallPassedPub.accept(moduleTestData[0].passed && moduleTestData[1].passed && moduleTestData[2].passed && moduleTestData[3].passed);
    }


    public enum Stage{
        ANGLE_FORWARD, //point the modules forward, prompt visual check
        ANGLE_90, //point the modules at a 90 degree angle, prompt visual check, look at current draw & module cohesiveness, duration of motion, position tracking accuracy
        SPIN_TEST, //ramp modules to 100% speed, check rotational rate of encoder
        DONE
    }

    public static class ModuleTestData{
        public double cohesion = 0;
        public double angleAccumCurrent = 0;
        public double anglePeakCurrent = 0;
        public double duration = 0;
        public double angleError = 0;
        public double rotationalRate = 0;
        public double driveError = 0;
        public double driveAccumCurrent = 0;
        public double score = 0;
        public boolean passed = true;
    }

    public static class ModuleIterData{
        public final double angleCurrentDraw;
        public final double driveCurrentDraw;
        public final double angle;
        public final double angleError;
        public final double rotationalRate;
        public final double driveError;

        public ModuleIterData(
            double angleCurrentDraw, 
            double driveCurrentDraw, 
            double angle, 
            double angleError, 
            double rotationalRate, 
            double driveError
        ){
            this.angleCurrentDraw = angleCurrentDraw;
            this.driveCurrentDraw = driveCurrentDraw;
            this.angle = angle;
            this.angleError = angleError;
            this.rotationalRate = rotationalRate;
            this.driveError = driveError;
        }

        public static ModuleIterData getModuleData(int i, PeaccefulSwerve swerve, double setpoint){
            return new ModuleIterData(
                swerve.getModuleSteerCurrent(i),
                swerve.getModuleDriveCurrent(i),
                Math.abs(swerve.getModuleAngle(i)),
                Math.abs(swerve.getModuleAngle(i)) - setpoint,
                swerve.getModuleRotationalRate(i),
                swerve.getModuleDriveError(i)
            );
        }
    }
}
