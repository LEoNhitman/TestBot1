# `Main.java`

**Package:** `frc.robot`  
**Role:** Java entry point for the WPILib robot program.

## Responsibility

`Main.java` is the program entry point. It hands control to WPILib's robot runtime and identifies `Robot` as the class that implements the robot lifecycle.

In a standard WPILib Java project, this file is intentionally small. It is not where drivetrain logic, controller mappings, or subsystem construction belongs.

## Relationship to other files

- Starts the WPILib runtime.
- The runtime constructs and manages the `Robot` class.
- `Robot` creates the `RobotContainer`, which constructs the drivetrain subsystem and configures commands.

## Why it is separate

Keeping the entry point separate from robot behavior follows the standard WPILib project structure. The entry point should remain stable while robot behavior is developed in `Robot`, `RobotContainer`, subsystems, commands, and IO classes.

## Maintenance

There is normally little reason to modify this file unless the project's startup structure changes. The exact source should remain the WPILib-generated entry point for the project's selected WPILib version.
