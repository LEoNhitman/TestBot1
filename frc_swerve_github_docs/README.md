# FRC Swerve Drive Robot

This repository contains a command-based FRC robot project built around a four-module swerve drivetrain. The drivetrain is organized so that the same subsystem and module logic can work with either real hardware or simplified software simulation.

> **Project status:** The current project has a compiling swerve subsystem, separate real/sim IO implementations for modules and gyro, and a simulation that launches. The simulation is still simplified and is not a complete physics model.

## What the robot software currently contains

- WPILib's `TimedRobot` lifecycle and command scheduler
- A `RobotContainer` that owns the drivetrain and driver-controller bindings
- A `SwerveSubsystem` that coordinates four modules and performs chassis-to-module kinematics
- A `SwerveModule` that optimizes module states and controls steering and drive through an IO interface
- Separate real and simulated implementations for module and gyro IO
- SmartDashboard telemetry for heading, gyro connection, and module steering angles

## Repository documentation

Detailed documentation is in [`docs/`](docs/):

- [Architecture overview](docs/architecture.md)
- [Main.java](docs/Main.md)
- [Robot.java](docs/Robot.md)
- [RobotContainer.java](docs/RobotContainer.md)
- [SwerveConstants.java](docs/SwerveConstants.md)
- [SwerveSubsystem.java](docs/SwerveSubsystem.md)
- [SwerveModule.java](docs/SwerveModule.md)
- [SwerveModuleIO.java](docs/SwerveModuleIO.md)
- [SwerveModuleIOReal.java](docs/SwerveModuleIOReal.md)
- [SwerveModuleIOSim.java](docs/SwerveModuleIOSim.md)
- [GyroIO.java](docs/GyroIO.md)
- [GyroIOReal.java](docs/GyroIOReal.md)
- [GyroIOSim.java](docs/GyroIOSim.md)

## High-level architecture

```text
Main
 └── Robot (TimedRobot)
      ├── CommandScheduler
      └── RobotContainer
           ├── Driver controller
           └── SwerveSubsystem
                ├── SwerveDriveKinematics
                ├── GyroIO
                │    ├── GyroIOReal -> Pigeon2
                │    └── GyroIOSim -> simulated heading
                └── Four SwerveModules
                     └── SwerveModuleIO
                          ├── SwerveModuleIOReal
                          │    ├── TalonFX drive motor
                          │    ├── SPARK MAX steering motor
                          │    └── CANcoder absolute encoder
                          └── SwerveModuleIOSim
```

## How a driver input reaches the motors

1. `Robot` runs WPILib's `CommandScheduler` every robot loop.
2. `RobotContainer` installs a default command on `SwerveSubsystem`.
3. The default command calls `driveWithController()`.
4. Controller axes are read, inverted where appropriate, and passed through a deadband.
5. Inputs are scaled to the configured maximum linear and angular speeds.
6. `SwerveSubsystem.drive()` creates a `ChassisSpeeds` object. For field-relative driving, the current gyro heading is used to transform field coordinates into robot coordinates.
7. `SwerveDriveKinematics` calculates the desired speed and angle for each module.
8. Wheel speeds are desaturated so no module exceeds the configured maximum.
9. Each `SwerveModule` optimizes its requested state, calculates steering output with a PID controller, and sends drive velocity and steering output through its `SwerveModuleIO`.
10. The selected IO implementation talks to hardware or updates the simplified simulation.

## Important implementation notes

- **Hardware constants must be verified.** CAN IDs, encoder offsets, gear ratios, wheel dimensions, module positions, motor inversions, and controller gains must be checked against the team's actual robot configuration before hardware operation.
- The constants currently in the source are project values/placeholders, not proof that the robot wiring matches them.
- `SwerveModuleIOSim` is a basic software model. It integrates commanded drive velocity into distance and steering output into a steering position; it does not model real motor dynamics, current draw, voltage behavior, wheel-ground interaction, or chassis physics.
- `GyroIOSim` integrates the commanded chassis angular velocity supplied by the subsystem. It is not yet driven by measured simulated wheel motion.
- The current project does not yet implement full drivetrain odometry, pose estimation, vision fusion, or a path-planning autonomous system.
- The controller is configured at port `0`. A physical controller is not required to compile the project or launch the Sim GUI.
- The temporary gyro test discussed during development is not part of the documented permanent `RobotContainer` code.

## Build and simulation

Use the WPILib VS Code extension to build and launch the robot project. The project has been built successfully and launched in WPILib simulation during development.

For a real robot, perform configuration checks and all testing under the team's normal robot-safety procedures. A successful software build does not verify that motor IDs, encoder offsets, motor directions, or control gains are correct.

## Documentation conventions

Each file-specific page explains:
- The file's responsibility
- Its important fields and methods
- How it interacts with other files
- The current limitations and assumptions

The documentation describes the architecture and code developed in this project. If the source code changes, update the corresponding page so the documentation stays synchronized.
