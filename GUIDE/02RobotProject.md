# Setting up your robot project
First I'll introduce you to Visual Studio Code, then I'll go over creating a blank project from scratch, then we'll talk about creating a project based on this project. Finally, we'll go over vendor libraries.
## Your New Home, a.k.a. VS Code
It's important to get comfortable in Visual Studio Code - you'll be spending hundreds of hours there in the upcoming months. Foundationally, VS Code is a text editor, but it integrates a lot of other features. I'm going to go over them just enough to make your brain hurt.

The key elements of VS Code is the text editor in the middle, the file explorer, the command palatte, source control, and the terminal. VS Code also supports thousands of extensions - it's the current industry standard, and can be used for just about anything programming related.

### Text Editor:
The VS Code editor has several helpful features. Firstly, all the colors! This is called syntax highlighting. It just colors each type of word differently based on the language your using. Once you're used to it, this helps you focus on the important parts when you're reading code. It doesn't do anything else.

Secondly, autocomplete. This is your best freind. It shows you two things. One is to show all the variable names you've defined based on what you've already typed. This way, you don't need to remember exactly what your variables are called. Even more helpful is this: it will show you every possible variable and function that is available in objects you reference. This won't really make sense until you have an understanding of object oriented programming. Basically, though, it shows you everything the software library you're using is capable of. Once you get to writing code, scroll all the way through some of the autocomplete lists. It's actually a good way to learn.

An honourable mention is CTRL+click. In our robot code, there are lots of places where we reference other files in the project. In Java, a "class" and a "file" are the same thing - classes are files. You can simply control+click on the name of a class, and it will take you right to the file the class is defined in. **This works for EXTERNAL LIBRARIES TOO!** That means that you can just control+click on a class your interested in and look at *WPILib's internal workings*. Ignore this for now, but come back to it! It's an awesome way to learn what is actually happening at a deeper level, instead of having WPILib be a black box that you don't known anything about.

### File Explorer:
Each VS Code window is focused on a single folder, which can be any folder on your computer. The file explorer shows all the files in the open folder, which makes it easy to navigate your project. It is located at the top of the "primary side bar" on the left. By default anyway - VS Code is very customizable and my editor layout is completely different from the default.

### Command Pallate:
All the things VS Code is capable of are defined as "commands" that you can access from the command pallate. The easiest way to open the command pallate is by pressing Ctrl+Shift+P. After opening the command pallate, just type to search for a command. Most importantly for us, **This is where the option to deploy code is**, and also to create a new WPILib project.

**In WPILib VS Code, there is a little WPILib icon in the top right.** This brings up a command pallate that is filtered to only WPILib commands. Open this now and scroll through it!

### Source Control:
We will go over this in depth in Section 4. Simply put, it's a very fancy "undo" button. It makes sure you always have a working version of the code.

### The Terminal:
The terminal is foundational to programming. In the Windows operating system there are two different terminals, Command Prompt and PowerShell (which adds additional functionality). Either is fine for us. **The terminal is the *Non-graphical* part of the operating system**. Think about how old computers had no mouse - all they had was a keyboard. The text-only software that was available for those old computers is still on **every computer**. The graphical part of the operating system is built *on top* of the text-based part. Using the terminal lets you tap into the core functionality of your computer that isn't always accessible through the graphical side.

How does that work? Well, it's like a simple programming language. It revolves around running text based applications called commands. For example, the "ls" command prints out all the files in the current directory. Or the "mkdir" command creates a new *directory* (folder). To run a command you just type the name of the command and press enter: "ls\<enter\>". 

You can also pass information into commands by putting it after the command name. What happens to this information is command-specific. For example: "mkdir myfolder" creates a folder called myfolder. This text after the command is called "arguments". Many commands have special arguments that modify their functionality. These arguments are usually preceded by either a dash "-" or two dashes "--".

**DON'T PANIC!** ALL of the command line stuff for FRC is automatically handled by WPILib. I just don't want you to be completely in the dark on what happens when you press *deploy*.

Since the terminal is an integral part of programming, there is one built directly into VS Code in the bottom pane. You can close this pane, and it will re-appear when it is used by WPILib, like when deploying code.

***If anything goes wrong when deploying your code, error details will be displayed in the terminal***

### Errors
VS Code automatically detects syntax errors (things your programming language doesn't allow) in your software. This is immensly useful. In the status bar at the very bototm of VS Code, there is an indicator that shows the number of errors, warnings, and information messages. **If there are any errors, you won't be able to deploy your code!** Most warnings can be ignored till the end of the development process; usually they point to useless code that can be removed.

Errors are marked in the editor with a sqiggly underline, like a misspelled word in normal text editors. If you hover your mouse over the error, you'll get an explanation of what the error is for. If you do not understand the message, google it.

For some errors, there are "quick fix" options where the editor tries to figure out how to fix the issue. You can access them by putting the cursor on the error and either pressing Ctrl+. or clicking the little lightbulb that appears to the left.

***Don't use a quick fix unless you know what it will do!***

The thing quick fix is awesome for is finding imports - we'll get to that later.

### GET COMFORTABLE
Before you go on, I strongly reccomend that you spend some time messing around in VS Code. Download this robot project's source code (as a zip) from [here](https://github.com/Operation-P-E-A-C-C-E-Robotics/base-swerve/releases). Unzip it, and open it in VS Code (File>Open Folder). Do whatever you want, you won't hurt anything. There's a lot I haven't gone over here, like the (non-command) pallate, outline view, searching, etc.

## Creating a Project!
To create a project, just open the command pallate and type "create a new project" and press enter. (you don't have to type the whole thing, as soon as it's at the top of the list you can press enter). It will open a little form to fill out. 

Click on Select a project type. Choose template>java>Command Robot. Choose a folder, name your project, and type in your team number. Click Generate Project. That's it!

While we're here, let's talk about naming projects. For official robot code, our team uses the simple standard of "frc-\<year>". For personal projects while you're learning, name it whatever you want (as long as it's valid of course).

## Using this Project!
The best way to build code off of this project is to use a git feature called "Forking". To do this, just go to the [home page of this repository](https://github.com/Operation-P-E-A-C-C-E-Robotics/base-swerve), then click the fork button. Name the new repository either "frc-\<year>" for official code, or whatever you want for unofficial code.

Now, use git to clone the new repo. We'll go over this in detail in Section 04, but here's how to do it. Open the command pallate with ctrl+shift+p, type in "clone", and press enter. Then, copy the url to the new github repo you created, and paste it into the bar that pops up. Press enter. That's it!

**It will clone into the folder that's open in VS Code, so make sure it's a folder you want the project in!**

# Vendor Libraries
A vendor library is a bunch of code written by some one else to make our lives easier. A lot of FRC devices, like motor controllers, use special protocols that *no one* wants to deal with. For example, the CAN Bus protocol that connects nearly all the devices on our robot. Implementing it requires some very complicated low-level code that I do not understand. Forunately this is handled by our wonderful libraries.

Most vendor libraries are from manufacturers of the hardware we use, like CTRE and Rev. They write libraries that make it incredibly easy to use their devices without needing to understand a single thing about the protocol. CTRE's library is called Phoenix and Rev's library is called REVLib.

There are also libraries that aren't for a physical product but simply help *do stuff*. The most common of these is PathPlannerLib which helps with autos.

**To install vendor libraries** open the command pallate, type in "vendor libraries", and press enter. Choose "install new libraries (online)" and then paste in the url for the library. 

Here is where to find the URLs for common libraries:
- [Phoenix 6](https://v6.docs.ctr-electronics.com/en/latest/docs/installation/installation-frc.html) (choose "online" instead of offline)
- [REVLib](https://docs.revrobotics.com/ion-control-system/sw/revlib#c-and-java-installation)
- [PathPlannerLib](https://pathplanner.dev/pplib-getting-started.html)
- [photonlib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html)

These libraries are already installed in this project, but will probably need to be updated.

# Project Migration
Each year, the new version of WPILib is slightly different from the old version. You ***MUST*** import any older project you want to use to the new version. If you don't, you won't be able to deploy code to any up-to-date RoboRIO. Additionally, vendor libraries sometimes are only compatible with current firmware. In general it's best to make sure that everything is up to date on both the hardware and software side, to minimize headaches and wasted time. This is really the worst part of FRC programming

Occasionally there will be changes to the libraries that actually affect how you are able to write code, so you'll have to change a few things in a project you migrate.

I'll probably handle moving this codebase from 2024 wpilib to 2025.

## [1<<---](01DriverStation.md)[2][--->>3](03ProgrammingSkills.md)
# [INDEX](/README.md)
1. [Setting up your driver station](/GUIDE/01DriverStation.md)
2. [Setting up your robot project](/GUIDE/02RobotProject.md)
3. [Basic programming skills](/GUIDE/03ProgrammingSkills.md)
4. [Using GIT & GitHub](/GUIDE/04GIT&Github.md)
5. [Getting a swerve running (using this project)](/GUIDE/05Swerve.md)
6. [End Effectors, Intakes, and Pneumatics](/GUIDE/06BasicMechs.md)
7. [Motion control 101](/GUIDE/07MotionControl101.md)
8. [How to program an elevator](/GUIDE/08Elevator.md)
9. [How to program an arm](/GUIDE/09Arm.md)
10. [How to program a shooter](/GUIDE/10Shooter.md)
11. [Kinematics 101](/GUIDE/11Kinematics101.md)
12. [Creating good OIs & automation](/GUIDE/12Automation&OI.md)
13. [Creating an autonomous](/GUIDE/13Autonomous.md)
14. [Telemetry & Useful Data Sources](/GUIDE/14Telemetry&Data.md)
15. [Programming Using State Machines](/GUIDE/15StateMachines.md)
16. [Available debugging resources.](/GUIDE/16Debugging.md)
17. [Competitions and Sanity](/GUIDE/17Competition.md)