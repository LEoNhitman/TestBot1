# `Robot.java`

**Package:** `frc.robot`  
**Base class:** `TimedRobot`  
**Role:** Owns the top-level robot lifecycle and runs the command scheduler.

## Responsibility

`Robot` is the WPILib lifecycle class. It receives callbacks when the robot starts, changes modes, and runs periodic loops.

The project uses `TimedRobot`, which is the normal WPILib base class for a timed robot program. It does not need to be replaced with a different robot superclass to use command-based programming.

## Command scheduler

The key command-based integration is:

```java
CommandScheduler.getInstance().run();
```

This is called from `robotPeriodic()`. It allows WPILib's command framework to:
- Poll registered button bindings
- Schedule commands whose conditions become true
- Execute scheduled commands
- Call subsystem periodic methods
- Finish commands whose completion conditions are met
- Run default commands when their required subsystems are free

The scheduler should be run once per robot loop.

## RobotContainer

The `Robot` class constructs a `RobotContainer`. The container is responsible for constructing subsystems, controllers, commands, and button bindings.

This separation keeps the lifecycle class small and keeps robot-specific wiring in `RobotContainer`.

## Mode transitions

The project's standard command-based lifecycle schedules the autonomous command during autonomous mode and cancels it when teleop begins. The exact implementation should be kept synchronized with the current `Robot.java` source.

## What does not belong here

Drivetrain kinematics, motor configuration, module control, controller-axis calculations, and button-to-command mappings belong in their respective subsystem, IO, and container classes—not in `Robot.java`.
