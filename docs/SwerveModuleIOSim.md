# `SwerveModuleIOSim.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Implements:** `SwerveModuleIO`  
**Role:** Provides a lightweight software-only implementation of a swerve module.

## Responsibility

This class lets the drivetrain subsystem and module logic run in simulation without constructing TalonFX, SparkMax, or CANcoder hardware objects.

It implements the same `SwerveModuleIO` interface as `SwerveModuleIOReal`.

## Stored state

The simulation stores:
- Drive velocity in meters per second
- Drive position in meters
- Steering position in rotations
- Steering output
- The timestamp of the previous update

## Time integration

`updateInputs()` reads the current FPGA timestamp and calculates elapsed time since the previous call:

```text
deltaTime = currentTimestamp - lastTimestamp
```

It then updates the simulated state:

```text
drivePosition += driveVelocity × deltaTime
steerPosition += steerOutput × 0.5 × deltaTime
```

The steering position is wrapped into [-0.5, 0.5) rotations. The resulting values are copied into the shared `SwerveModuleIOInputs` structure.

The simulation marks its devices as connected and reports zero voltage/current values.

## Output methods

- `setDriveVelocity(speed)` stores the requested speed. The next input update integrates that speed into distance.
- `setSteerOutput(output)` clamps and stores the requested steering output.
- `stop()` sets both simulated drive velocity and steering output to zero.

## Limitations

This is an intentionally simplified model. It does not simulate:
- Motor voltage/current behavior
- Motor acceleration, inertia, or friction
- Gearbox dynamics
- Steering motor feedback dynamics
- Wheel-ground interaction or slipping
- Chassis translation or rotation
- Battery voltage sag
- Realistic sensor noise or device faults

The steering rate multiplier (`0.5`) is an arbitrary simplified rate, not a calibrated motor model. This class is useful for exercising software flow and checking that the subsystem can run without hardware, but it should not be treated as a predictive model of real robot movement.
