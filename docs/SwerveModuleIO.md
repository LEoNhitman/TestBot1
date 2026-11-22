# `SwerveModuleIO.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Type:** Interface  
**Role:** Defines the contract between `SwerveModule` and the module's underlying hardware or simulation.

## Why this interface exists

`SwerveModule` needs a consistent way to:
- Read sensor values
- Request a drive velocity
- Request a steering output
- Stop the module

It should not need to know which motor controller or simulation implementation provides those operations. `SwerveModuleIO` defines that shared contract.

## `SwerveModuleIOInputs`

The nested input class is a container for the latest measured or simulated values.

The project input structure includes:
- `driveVelocityMetersPerSecond`: measured/simulated wheel linear speed
- `drivePositionMeters`: accumulated wheel travel
- `steerPositionRotations`: steering angle represented in rotations
- `driveAppliedVolts`: drive motor voltage
- `steerAppliedOutput`: normalized steering motor output
- `driveCurrentAmps`: drive motor current
- `steerCurrentAmps`: steering motor current
- `driveConnected`: drive device/signal connection status
- `steerConnected`: steering device connection status
- `encoderConnected`: absolute encoder connection status

The IO implementation updates this structure. `SwerveModule` reads it.

## Interface methods

- `updateInputs(inputs)`: refreshes the input structure.
- `setDriveVelocity(metersPerSecond)`: requests drive wheel speed.
- `setSteerOutput(output)`: requests a steering motor output.
- `stop()`: stops the module outputs.

The current interface uses default no-op method bodies. This makes it easier to build the architecture incrementally, but it also means a missing override may silently do nothing. As the design stabilizes, consider making required operations abstract so every implementation must provide them.

## Implementations

- `SwerveModuleIOReal` uses CTRE and REV hardware APIs.
- `SwerveModuleIOSim` updates software variables and integrates them over time.

Both implementations expose the same interface to `SwerveModule`.
