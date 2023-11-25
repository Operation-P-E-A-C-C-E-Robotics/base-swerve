package frc.lib.swerve;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveSelfCheck extends Command {
    private PeaccefulSwerve swerve;
    private NetworkTable swerveTestNT = NetworkTableInstance.getDefault().getTable("Test").getSubTable("Swerve Self Check");

    private StringPublisher message = swerveTestNT.getStringTopic("Message").publish();
    private BooleanSubscriber continueButton = swerveTestNT.getBooleanTopic("Continue").subscribe(false);
    private BooleanPublisher continueButtonReset = swerveTestNT.getBooleanTopic("Continue").publish();

    //how closely all the modules are moving together
    private DoublePublisher module0CohesionPub = swerveTestNT.getDoubleTopic("Module 0 Cohesion").publish();
    private DoublePublisher module1CohesionPub = swerveTestNT.getDoubleTopic("Module 1 Cohesion").publish();
    private DoublePublisher module2CohesionPub = swerveTestNT.getDoubleTopic("Module 2 Cohesion").publish();
    private DoublePublisher module3CohesionPub = swerveTestNT.getDoubleTopic("Module 3 Cohesion").publish();

    //peak current draw
    private DoublePublisher module0CurrentPub = swerveTestNT.getDoubleTopic("Module 0 Current").publish();
    private DoublePublisher module1CurrentPub = swerveTestNT.getDoubleTopic("Module 1 Current").publish();
    private DoublePublisher module2CurrentPub = swerveTestNT.getDoubleTopic("Module 2 Current").publish();
    private DoublePublisher module3CurrentPub = swerveTestNT.getDoubleTopic("Module 3 Current").publish();

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

    private final ModuleTestData[] moduleTestData = new ModuleTestData[4];

    //TODO arbitrary selections
    private final double MAX_CURRENT = 20; //amps
    private final double MIN_COHESION = 100; //arbitrary
    private final double MAX_DURATION_90 = 0.1; //seconds
    private final double MAX_ACCUM_ERROR_ANGLE = 400; //arbitrary
    private final double MIN_ROTATIONAL_RATE = 100; //degrees per second
    private final double MAX_ACCUM_ERROR_DRIVE = 100; //arbitrary

    private final double MAX_DIFFERENCE_BETWEEN_CANCODER_AND_MOTOR_ENCODER = 0.1; //radians

    /**
     * Swerve Self Tester! :D
     * First, it will orient the wheels forward, then at 90 degrees, then spin them, then drive 1 meter forward
     * It checks for module current draw, how similar the module's motion is ("cohesion"), and how well the module tracks it's angle.
     * @param swerve
     * @param requirements
     */
    public SwerveSelfCheck(PeaccefulSwerve swerve, Subsystem... requirements){
        this.swerve = swerve;

        addRequirements(requirements);
    }



    public enum Stage{
        WAITING_FOR_START, //waiting for the user to press the button
        ANGLE_FORWARD, //point the modules forward, prompt visual check, look at current draw
        ANGLE_90, //point the modules at a 90 degree angle, prompt visual check, look at current draw & module cohesiveness, duration of motion, position tracking accuracy
        SPIN_TEST, //ramp modules to 100% speed, check rotational rate of encoder, cancoder agreement with motor encoder, sensor noise
        DRIVE_VELOCITY_TEST //check velocity controller tracking error
    }

    public static class ModuleTestData{
        public double cohesion = 0;
        public double current = 0;
        public double duration = 0;
        public double angleError = 0;
        public double rotationalRate = 0;
        public double driveError = 0;
        public double score = 0;
        public boolean passed = false;
    }
}
