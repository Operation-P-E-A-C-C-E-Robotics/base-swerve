# Code Structure
- [Setting Up](Setting_Up.md) A New Robot
- [Organization](Organization.md) Principles
- [Code Style](Code_Style.md)
- [Folder Structure](Folder_Structure.md)

The software is designed around the WPILib Command Based framework. The base robot contains only the DriveTrain subsystem and PeaccyDrive command. As WPILib has plenty of docs on how to use command-based, I won't repeat it here. There is also a libs folder (src/main/java/frc/lib), which has robot-independed utilities and helpers, such as motion profiling and various math utilities. See the sub-documents for more details