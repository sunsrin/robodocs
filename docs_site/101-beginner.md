# 101: Welcome to FRC Software

This guide introduces Command-Based Programming.

## What is Command-Based Programming?
WPILib's Command-Based framework splits the robot code into **Subsystems** (hardware) and **Commands** (actions).

- **Subsystems:** Represent parts of the robot (e.g., Drive, Launcher). They handle the hardware logic.
- **Commands:** Represent actions (e.g., Shoot, Drive Forward). They use subsystems to do work.
- **RobotContainer:** The glue that binds Commands to Controller buttons.

## Where does the code start?
The robot boots up in `Robot.java`, which instantiates `RobotContainer.java`. Look inside `RobotContainer` to see all the subsystems being created!
