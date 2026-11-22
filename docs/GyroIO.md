# `GyroIO.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Type:** Interface  
**Role:** Defines the shared interface for real and simulated gyroscopes.

## Why this interface exists

The drivetrain needs a heading measurement and a way to reset the gyro. It should not need to know whether the heading comes from a physical Pigeon 2 or a software simulation.

`GyroIO` provides that common contract.

## `GyroIOInputs`

The nested input class stores:
- `yawDegrees`: the latest gyro yaw value in degrees
- `connected`: whether the implementation reports that the gyro signal is available

The subsystem owns a `GyroIOInputs` instance and passes it to `updateInputs()`.

## Methods

- `updateInputs(inputs)`: updates the stored yaw and connection status.
- `reset()`: resets the gyro heading.

The interface currently provides default no-op implementations. This supports incremental development, but a missing override could silently do nothing. Consider making required methods abstract once all implementations are established.

## Implementations

- `GyroIOReal` reads and resets a CTRE Pigeon 2.
- `GyroIOSim` stores and integrates a simulated yaw value.

The subsystem selects the implementation at construction time.
