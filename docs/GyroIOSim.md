# `GyroIOSim.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Implements:** `GyroIO`  
**Role:** Provides a software-only gyro for simulation.

## Stored state

- `yawDegrees`: the simulated heading
- `angularVelocityRadiansPerSecond`: the angular velocity supplied by the drivetrain
- `lastTimestamp`: timestamp of the previous update

## `setAngularVelocity(...)`

The subsystem calls this method with `ChassisSpeeds.omegaRadiansPerSecond`. The value is stored in radians per second.

This is a simplified connection between the commanded chassis rotation and the simulated gyro.

## `updateInputs(...)`

The method:
1. Reads the current FPGA timestamp.
2. Calculates elapsed time since the last update.
3. Updates `lastTimestamp`.
4. Integrates angular velocity over elapsed time.
5. Converts the resulting angular displacement from radians to degrees.
6. Wraps the heading into [-180, 180) degrees.
7. Copies the heading and a simulated connected status into `GyroIOInputs`.

The core integration is:

```text
yawDegrees += degrees(angularVelocityRadiansPerSecond × deltaTime)
```

## `reset()`

Sets the simulated yaw to zero degrees.

## Limitations

This simulation integrates the commanded angular velocity provided by `SwerveSubsystem`. It does not calculate angular velocity from module sensor readings, wheel contact, or chassis physics. It also does not model gyro drift, noise, bias, latency, or disconnection.

It is suitable for basic software testing of heading-dependent logic, but it is not a replacement for a full drivetrain simulation.
