# Drivetrain Architecture

## Design goal

The drivetrain uses a layered, command-based architecture. High-level code describes what the robot should do, while lower-level classes translate those requests into module commands or hardware operations.

A major design choice is the **IO (input/output) abstraction**. The subsystem and module logic depend on interfaces rather than directly depending on vendor hardware classes. This allows the same control structure to select either real hardware IO or simulation IO.

## Layers

### 1. Robot lifecycle

`Main.java` starts the WPILib robot runtime. `Robot.java` provides the `TimedRobot` lifecycle and runs `CommandScheduler` during `robotPeriodic()`.

### 2. RobotContainer and commands

`RobotContainer.java` constructs the `SwerveSubsystem`, creates the driver controller, installs the drivetrain's default command, and registers button bindings.

The default command continuously calls `driveWithController()` while the subsystem is not being used by another command. Its end action calls `stopModules()`.

### 3. SwerveSubsystem

`SwerveSubsystem` is the drivetrain coordinator. It owns:
- Four `SwerveModule` objects
- One `GyroIO` implementation
- `SwerveDriveKinematics`
- The high-level `drive`, `crabwalk`, `stopModules`, heading, and gyro-reset operations

The subsystem converts chassis motion into four module states. It does not need to know whether a module is backed by a TalonFX/SparkMax/CANcoder or by a software simulation.

### 4. SwerveModule

Each `SwerveModule` represents one physical corner of the drivetrain. It:
- Reads module sensor inputs through `SwerveModuleIO`
- Optimizes the requested state to reduce unnecessary steering rotation
- Uses a PID controller to calculate steering output
- Sends the desired drive velocity and steering output to the IO implementation
- Publishes steering-angle telemetry

### 5. IO interfaces and implementations

`SwerveModuleIO` defines the operations and sensor-input structure used by a module. `SwerveModuleIOReal` implements those operations using vendor hardware. `SwerveModuleIOSim` implements them using software state.

Likewise, `GyroIO` defines gyro input and reset operations. `GyroIOReal` uses a CTRE Pigeon 2, while `GyroIOSim` maintains a simulated yaw value.

## Runtime selection

When `SwerveSubsystem` is constructed, it checks `RobotBase.isSimulation()`.

- In simulation, it creates four `SwerveModuleIOSim` instances and a `GyroIOSim`.
- Otherwise, it creates four `SwerveModuleIOReal` instances using the configured CAN IDs and offsets, plus a `GyroIOReal`.

This selection is centralized in the subsystem constructor. The rest of the drivetrain code uses the same `SwerveModule` and `GyroIO` abstractions.

## Drive calculation flow

The `drive(forward, strafe, rotation, fieldRelative)` method accepts:
- Forward linear velocity in meters per second
- Strafe linear velocity in meters per second
- Angular velocity in radians per second
- A flag selecting field-relative or robot-relative interpretation

For robot-relative driving, those values are passed directly into `ChassisSpeeds`.

For field-relative driving, `ChassisSpeeds.fromFieldRelativeSpeeds()` uses the current gyro heading to rotate the requested field-coordinate speeds into the robot's coordinate frame.

`SwerveDriveKinematics.toSwerveModuleStates()` converts chassis motion into a desired speed and angle for each wheel. `desaturateWheelSpeeds()` scales the wheel speeds together if needed so the maximum configured wheel speed is not exceeded.

The four resulting states are passed to the four modules in the order used to construct the kinematics object: front-left, front-right, back-left, back-right.

## Coordinate and unit conventions

- Linear speeds are represented in meters per second.
- Angular speeds are represented in radians per second.
- Gyro input is represented in degrees and converted to `Rotation2d`.
- Module steering sensor positions are represented in rotations by the IO input structure; the module converts rotations to radians or degrees for control and telemetry.
- Module translations are converted from inches to meters when constructing kinematics.

The signs of forward, strafe, rotation, and gyro yaw must be consistent with the team's chosen WPILib coordinate convention and the physical wiring/mounting of the robot. Verify them before operating hardware.

## Current limitations

- The module simulation is a simple integrator, not a motor or chassis physics simulation.
- The gyro simulation integrates commanded angular velocity. It does not infer rotation from simulated wheel states.
- No drivetrain odometry or pose estimator is currently present.
- No vision measurement fusion or path-planning autonomous integration is currently present.
- The current steering PID and drive velocity control values are starting values and must be tuned and validated for the actual robot.
- Hardware IDs, offsets, ratios, and geometry must be confirmed against the current robot configuration.
