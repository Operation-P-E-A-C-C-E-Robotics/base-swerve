# Organization
- The code is organized according to wpilib's command based framework.
- Each subsystem should have a subfolder in the commands folder for it's commands.
- Additional files that will be reused can go in the lib folder, under an appropriate subfolder.
- Constants should be in per-subsystem subclassess
- No joysticks or buttons should be owned by commands, they should be passed in through suppliers to make the OI easier to change.
- Code should be split up in two ways: either into classes that reflect a real-world counterpart or classes that solve a specific problem (where no real-world system is applicable)
- Variable names should be UNDERSTANDABLE, and concice but NOT ABBREVIATED. See [Code Style](Code_Style.md)

## Current Folder Structure
- src/main/java/frc
  - lib: game-independent reusable components
    - motion: motion profiling tools
    - safety: code stability tools
    - sensors: utilities for interfacing with sensors more easily
    - swerve: swerve drive helpers
    - telemetry: robot telemetry for different systems.
  - robot: game-specific components
    - commandss: different actions the robot can perform
    - subsystems: interfaces for different independant robot systems.