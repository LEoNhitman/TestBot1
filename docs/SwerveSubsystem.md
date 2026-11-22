# `SwerveSubsystem.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Base class:** `SubsystemBase`  
**Role:** Coordinates the complete four-module swerve drivetrain.

## Responsibility

`SwerveSubsystem` is the high-level drivetrain interface used by commands and `RobotContainer`. It owns four `SwerveModule` objects, a gyro IO implementation, and a `SwerveDriveKinematics` object.

It accepts chassis-level movement requests and converts them into individual wheel states.

## Module objects

The subsystem owns:
- `frontLeft`
- `frontRight`
- `backLeft`
- `backRight`

The same module class is used in real and simulated operation. Only the IO implementation passed into each module changes.

## Gyro abstraction

The subsystem stores:
- `GyroIO gyroIO`
- `GyroIOInputs gyroInputs`
- A `GyroIOSim` reference used only when running in simulation

In the constructor, `RobotBase.isSimulation()` selects the implementation:
- Simulation: four `SwerveModuleIOSim` objects and one `GyroIOSim`
- Real robot: four `SwerveModuleIOReal` objects and one `GyroIOReal`

This prevents the simulation branch from constructing the physical Pigeon 2.

## Kinematics geometry

`SwerveDriveKinematics` is constructed with four `Translation2d` module locations. The order is front-left, front-right, back-left, back-right. The same order is used when applying the resulting states to the module objects.

The current dimensions are converted from inches to meters with `Units.inchesToMeters()`.

## `drive(...)`

Parameters:
- `forward`: forward chassis speed in meters per second
- `strafe`: sideways chassis speed in meters per second
- `rotation`: angular speed in radians per second
- `fieldRelative`: whether the translation inputs are field-relative

If field-relative mode is requested, `ChassisSpeeds.fromFieldRelativeSpeeds()` uses `getHeading()` to transform the request into robot-relative coordinates. Otherwise, a `ChassisSpeeds` object is created directly from the inputs.

In simulation, the subsystem passes the resulting `omegaRadiansPerSecond` to `GyroIOSim.setAngularVelocity()`. This allows the simplified simulated gyro to integrate the commanded angular velocity.

The method then:
1. Converts chassis speeds into four `SwerveModuleState` values.
2. Calls `desaturateWheelSpeeds()` to keep wheel speeds within the configured maximum.
3. Sends each state to its corresponding module.

## `crabwalk(...)`

`crabwalk()` creates chassis speeds from the supplied forward, strafe, and rotation values, converts them to module states, desaturates wheel speeds, and sends the states to all four modules.

In simulation it also updates the simulated gyro's commanded angular velocity.

## Other methods

- `calculateModuleStates(...)`: calculates module states and publishes their desired speeds and angles. It is a telemetry/helper method; it does not command the modules.
- `setAllModuleSpeed(...)`: sends the same drive speed request to all four modules. It does not calculate steering angles.
- `stopModules()`: calls `stop()` on all four modules and sets simulated gyro angular velocity to zero.
- `testKinematics()`: calls `calculateModuleStates(1.0, 0.0, 0.0)` as a basic calculation test.
- `getHeading()`: returns a `Rotation2d` based on the most recently updated gyro inputs. If the gyro input reports disconnected, it returns a zero rotation.
- `resetGyro()`: calls the gyro IO reset operation and immediately refreshes the stored gyro inputs.

## `periodic()`

WPILib calls this method from the command scheduler. It:
1. Refreshes the four module input structures.
2. Refreshes the gyro input structure.
3. Publishes robot heading and gyro connection status.
4. Calls each module's dashboard update method to publish steering angles.

This keeps sensor reading and telemetry updates in the subsystem's regular periodic loop.

## Simulation limitations

The module simulation is a basic integrator and the gyro simulation integrates commanded chassis angular velocity. Neither is a full physical simulation. In particular, the gyro does not derive heading from measured simulated wheel motion, and the drivetrain does not yet have odometry or a pose estimator.
