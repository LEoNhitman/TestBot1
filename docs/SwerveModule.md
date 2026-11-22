# `SwerveModule.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Role:** Controls one swerve module using a hardware-independent IO interface.

## Responsibility

A `SwerveModule` represents one wheel assembly. It does not create vendor motor or encoder objects itself. Instead, it receives a `SwerveModuleIO` implementation in its constructor.

That design lets the same steering and drive logic operate with either real hardware IO or simulated IO.

## Important fields

- `io`: the module's IO implementation.
- `inputs`: latest sensor and connection data populated by `io.updateInputs()`.
- `steerPID`: PID controller used to calculate steering motor output.

The steering PID is configured with continuous input from -0.5 to +0.5 rotations. This treats the steering angle as circular, so the controller can choose the shorter path across the wrap boundary.

## Constructor

The constructor stores the supplied IO object and enables continuous input on the steering PID controller.

## `updateInputs()`

Calls `io.updateInputs(inputs)` to refresh the module's input structure. The structure contains drive velocity, drive position, steering position, applied output/voltage, current, and connection-status fields.

## `setDesiredState(...)`

This is the main module-control method.

1. Refreshes module inputs.
2. Reads the current steering angle.
3. Calls `SwerveModuleState.optimize()` to choose an equivalent wheel orientation that can reduce unnecessary steering rotation. The wheel may reverse drive direction rather than rotate the steering assembly through a large angle.
4. Reads the current and target steering positions in rotations.
5. Uses the PID controller to calculate the steering output.
6. Clamps steering output to the range -0.5 to +0.5.
7. Sends steering output through `io.setSteerOutput()`.
8. Sends the optimized wheel speed through `setDriveSpeedMetersPerSecond()`.

The PID output is a normalized motor command in this implementation. Its gain and output limit must be validated and tuned for the actual steering mechanism.

## Angle accessors

- `getSteeringAngleRotations()`: returns the current steering position in rotations.
- `getSteeringAngleRadians()`: converts rotations to radians by multiplying by 2π.
- `getSteeringAngleDegrees()`: converts rotations to degrees by multiplying by 360.

The real IO implementation applies the absolute encoder offset and wraps the corrected reading into the interval [-0.5, 0.5).

## Drive and stop methods

- `setDriveSpeedMetersPerSecond(speed)`: delegates the requested wheel speed to `io.setDriveVelocity(speed)`.
- `stop()`: delegates to `io.stop()`, which should stop both drive and steering outputs.

## Dashboard

`updateDashboard(moduleName)` publishes the module's steering angle in degrees under a name based on the module label.

## Important distinction

The module contains control logic, but the IO implementation contains device-specific code. Do not add direct TalonFX, SparkMax, or CANcoder construction here; doing so would weaken the real/simulation separation.
