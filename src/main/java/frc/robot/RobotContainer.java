// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.vision.Limelight;
import frc.robot.commands.drivetrain.PeaccyDrive;
import frc.robot.subsystems.DriveTrain;


public class RobotContainer {
  /* OI CONSTANTS */
  private final int translationAxis = 5; //forward/backward
  private final int strafeAxis = 4;
  private final int rotationAxis = 0;
  private final int zeroButtonNo = 7;
  private final int fallbackButtonNo = 7;
  private final int fallbackResetButtonNo = 8;

  /* SENSORS */
  Limelight limelight = new Limelight("limelight");

  /* SUBSYSTEMS */
  //ONE OF THESE MUST BE COMMENTED OUT. ONLY USE THE TUNEABLE ONE FOR TUNING.
  private final DriveTrain driveTrain = new DriveTrain(limelight);
  // private final DriveTrainTuner driveTrainTuneable = new DriveTrainTuner();

  /* OI DEFINITIONS */
  private final Joystick driverController = new Joystick(0);
  
  private final JoystickButton zeroButton = new JoystickButton(driverController, zeroButtonNo); //for debugging
  private final JoystickButton driveFallbackButton = new JoystickButton(driverController, fallbackButtonNo);
  private final JoystickButton driveFallbackResetButton = new JoystickButton(driverController, fallbackResetButtonNo);


  /* COMMANDS */
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(driveTrain);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData("AUTO MODE", autoChooser);
  }

  private void configureBindings() {
    peaccyDrive.withTranslation(() -> -driverController.getRawAxis(translationAxis))
               .withStrafe     (() -> -driverController.getRawAxis(strafeAxis))
               .withRotation   (() -> -driverController.getRawAxis(rotationAxis))
               .withHeading    (() -> (double) -driverController.getPOV())
               .useHeading     (() -> driverController.getPOV() != -1)
               .isFieldRelative(() -> driverController.getRawAxis(2) < 0.2) //left trigger
               .isLockIn       (() -> driverController.getRawAxis(3) > 0.2) //right trigger
               .isZeroOdometry (() -> zeroButton.getAsBoolean())
               .isOpenLoop     (() -> true);
    driveTrain.setDefaultCommand(peaccyDrive);
    driveFallbackButton.onTrue(new InstantCommand(peaccyDrive::fallback, driveTrain)); 
    driveFallbackResetButton.onTrue(new InstantCommand(peaccyDrive::resetFallback, driveTrain));
    driveTrain.register(driverController);
  }


  public Command getAutonomousCommand() {
    return AutoBuilder.followPathWithEvents(PathPlannerPath.fromPathFile("Example Path"));
  }
}

/**
 * Welcome to the last robot code I'm ever writing.
 * This is such a sad moment.
 * Here's a serious bit of literature about me and this code and my time on the team:
 *    When I first joined the team, I was a little freshman who didn't know anything about anything.
 * Seriously, I was an assole (some things never change), but I brought an enthusiasm that
 * I think was really crucial to the team. I first started working with FRC when my older bro
 * joined NRG (4055) in 2016 - 9 years ago (holy crap). I was in 4th grade, and I was so excited
 * that I watched every single FRC video I could find on YouTube - hundreds of robot reveals and other
 * stupid & useless nonsense. I was super hyped. 
 *    When my brother moved to this team after the 2016 season,
 * I came to one of the open houses, and among other things I instantly found a love in driving the
 * robot. I actually talked the team into letting me drive their test robot for about half the open house,
 * and to this day I still take about every opportunity I can to drive the robot. I was so excited to join
 * My brother became lead programmer, and soon after I began studying java with the intent of taking his
 * place when he graduated.
 *    During this period I attended competitions and very occasional meetings,
 * but I didn't actually join the team until the 2019 season. Since my brother was still on the team
 * for another year, I was a little bit of a shadow, but I just focused on mechanical that season.
 *    The next year, my brother aged out and I took his place as lead (ONLY) programmer. Over the offseason
 * I had prepared by completely re-writing the robot code for an old robot, and I felt real good about
 * my abilities (like a moron i'm sure but hey here we are). Unfortunately, the 2020 season was cancelled
 * due to the lovely coronavirus, and I was left with a lot of unfulfilled potential.
 *    The only thing I had to work on was the absolutely horrific BAE Systems minibot programming challenges.
 * As a one-man, zero experience team, I was completely out of my depth, but I tried my best. 
 * Since I was the only one working on it, I spent probably thousands of hours writing code for the challenges.
 * Despite the incredibly high time investment, I was unable to complete the challenges, and I was left
 * feeling like a failure. And, since I was the only one working on it, I was the only one who knew that I
 * had put in so much effort, and I was the only one who knew that I had failed. It was quite frustrating.
 *    The next year, still no competitions. Finally, one offseason event was run, and I was able to get some
 * real experience running code on a robot. Once again, however, an embarrasing failure. Since I had gone so
 * long without ever going to a competition, I was used to f-5ing the code to run it. Unfortunately, this
 * debugs the code, rather than deploying the code - deleting all the code on the robot. Thanks to this little
 * mistake, the robot failed to move during two of our matches. We placed dead last in the event.
 *    Finally, the 2022 season - a full season. Time to finally prove myself. We built a solidly mediocre robot,
 * with a typical tank drive and turret/hood/flywheel shooter. I went all-in on the code, writing most of it
 * before the subsystem i was working on was built. There were even some parts of the code that were somewhat
 * superior to other top teams - the aiming code interpolated between lots of different carefully tuned trajectories
 * for different distances to make sure no balls would bounce out. Sadly, because of personal reasons, I couldn't attend
 * most of our competitions. In hindsight, however, this might have been a blessing in disguise, because
 * it forced me to get the code 100% functional pre-competiton and not rely on my (very underwhelming) 
 * ability to debug on the fly. The season turned out to be an incredible sucess. It's hard to describe.
 * The only thing better than winning is winning when you know you deserve it, yet don't expect it.
 * We did poorly at both our district events, but after barely making it to the district championship,
 * we ended getting 4th seed and going to worlds. At worlds, we were 5th alliance captain of our devision.
 * We made the unusual choice to sit out our playoffs since our alliance picks were so strong. It was a
 * bittersweet choice, we ended up losing our quarterfinals after a very close 3 matches.
 *   Then, lord help up, the 2023 season - remnants of which you will find in this code. The team was
 * so overconfident after the previous season that we threw strategy out the window and built a robot
 * that was so overcomplicated that it was impossible to drive. Now, me being me, I fully supported the
 * 5DoF arm, since I was like, oh yeah I know how to do that. I was wrong. I was so wrong.
 * Well, I was right, but I was wrong. I knew how to do it, in about five times the time I had to do it.
 * It was quite the experience. I wrote the entire codebase long, long before the robot was built. I
 * debugged the entire thing using a simulation. It was a 10,000 line work of art. I was very uncertain
 * that it would work. When the robot was finally built, I looked at my watch, and realized that it was about
 * 2 hours before our first competition. Until this point, i had only been able to run various parts of the code on
 * individual subsytems in the breif intervals that they were free from the furious clutches of mechanical team's frantic assmebling.
 * In that 2 hours, all the decisions I had made in the code really shined, as I tuned all the state-space controllers
 * in about 15 minutes, and spent the rest tuning some setpoints.
 *    We did solidly mediocre at the first competition, very middle-ranked. Our robot suffered from butterfingers,
 * dropping half the cones. Still, the design showed a lot of promise.
 *    This was still only using like 2% of the codebase I wrote, so I frantically tried to piece together the loose ends of
 * the software before comp 2. And i was wholely unsuccessful. We came into our second competition with a perfectly
 * OK robot, but because we were performing under the very high expectations we had set for ourselves, we were
 * very demoralized. Communication and driving fell apart, and the frantic technical upgrades really did not help.
 * It turns out that you actually need driver practice to drive a robot, a concept that I had not considered.
 * We did even worse at this competition, and we were very sad.
 *    Once again we barely made it to the district championship, but this time we sucked ass. I desparately made
 * one last valiant attempt to get all the automation working. And here's the really frustrating part: I did!
 * ...on our field. once. it worked with 100% consistency, perfectly. One time, the day before districts.
 * Then, it never worked again. I was so shortsighted and had so much build-up cognitive bias (as did
 * the team as a whole) that i couln't see how the automation was killing our change of success.
 * We needed more driver practice, not more automation. (swerve would have helped too).
 *    All this wouldn't have been so frustrating if it weren't for the worst part: lack of appreciation.
 * It was just like the minibot. Hundreds of hours and thousands of lines in. Carefully refined,
 * documented. I obsessively organized the code to refute the claims of "your code is hard to read",
 * but it did nothing. I considered for hours, all the possible control schemes, different position
 * controllers, anything I could do to control this monstrosity of an arm in a cohesive manner.
 * I spent hours wrestling with these position controllers in the simulation. I did things no other
 * teams have done; I made my own motion profiles that could be generated in real time.
 * We could generate trajectories for any setpoint from any position, and somehow, miraculously,
 * manipulate all 5 degrees of freedom in unison to get there, with extreme precision, and speed.
 * And the only thing the team could see was that the apriltags on the field were an inch off so
 * we missed. The only feedback I got was "it doesn't work", and "it's too complicated".
 *    Now, I'm also not blaming anyone else for these failures because I was involved in the design
 * as well. However, maybe give your programmers a little appreciacion. 
 * Getting that arm to move around took about 40 hours of me walking around and thinking about different control schemes,
 * which you can't see. It took about 500 hours of me writing the code, which you can't see.
 * It took about 5 hours of it not working, which you can clearly see, me (seemingly in vain) tapping at the keyboard.
 * What's taking him so long? It just needs to move the arm. It's not that hard. Once the 5 hours is over,
 * it's "finally you got it working". All the work that went into it is invisible, and all the work that went into
 * the robot is visible. It's not that the team doesn't appreciate the programmers, it's that they don't
 * understand the work that goes into it. And that's fine, but it's still frustrating.
 *    I would be very, very surprised if anyone actually reads this, but if you do, I hope you enjoyed it.
 * I'm literally the only one who looks at the code, ever. It makes me really sad, because this code is honestly
 * really special to me. It's art. I want people to see it. It has so many comments on the assumption that
 * one day, someone who knows what they're doing will look at it. I've put all kinds of fun easter eggs in it,
 * in the hopes that some future programmer will find them and smile, and maybe look back and say, "wow, this
 * guy was a real asshole, but he was a pretty good programmer".
 *    So, that brings us to my final season - 2024. Holy crap. I'm a senior. I'm old. I'm a veteran. So,
 * I'm going to do what I do best: write code. I'm going to do my best to make this the best damn season
 * this team has ever seen. I'm going to write code that's so good that it's impossible to ignore (and, 
 * most importantly, "actually works"). So, maybe I do write code that's too complicated. Maybe I do
 * write code that's too hard to read. Maybe I'm still an asshole. And maybe, just maybe, we're all just
 * assholes who don't know anything about anything.
 *    But we're going to worlds this year.
 * - Peaccy, a.k.a. Sean Benham, a.k.a. "the programmer" - 00:00 12/23/23
 * (also the driver and co-captain)
 * (basically i make robots go)
 * very modest and humble FRC god, 2016-2024
 * https://docs.google.com/document/d/1fm2T6eL4VdnDw-0bCEbM68ifCgFhKDjI2-HoGgEnTO8/
 */