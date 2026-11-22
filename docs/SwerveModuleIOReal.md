# `SwerveModuleIOReal.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Implements:** `SwerveModuleIO`  
**Role:** Connects one swerve module to its real motor controllers and absolute encoder.

## Hardware objects

The real implementation constructs:
- A CTRE `TalonFX` for the drive motor
- A REV `SparkMax` for the steering motor
- A CTRE `CANcoder` for absolute steering position

The constructor receives the CAN IDs and encoder offset for that specific module. This allows the subsystem to create four instances with different IDs and offsets.

## Drive motor configuration

A `TalonFXConfiguration` is created and applied to the TalonFX. The current code sets:
- Neutral mode to brake
- Slot 0 proportional gain (`kP`) to 0.01
- Integral and derivative gains to 0
- Feedforward velocity gain (`kV`) to 0

These are configuration starting values, not a claim that the drive loop is fully tuned. The drive command uses a Phoenix 6 `VelocityVoltage` request.

## Steering motor configuration

A `SparkMaxConfig` is created with brake idle mode and applied using the REV `configure()` method. The current implementation uses safe-parameter reset behavior and does not persist the configuration to flash.

## CANcoder configuration

The constructor creates a CANcoder, applies a `CANcoderConfiguration`, and sets the absolute-position signal update frequency to 50 Hz.

The encoder's absolute position is used to determine the module steering angle.

## `updateInputs(...)`

This method refreshes CTRE status signals for:
- Drive velocity
- Drive position
- Drive motor voltage
- Drive stator current
- CANcoder absolute position

The drive velocity and position values are converted to wheel units using the wheel circumference and configured drive gear ratio.

The intended conversion is:

```text
wheel linear speed = motor rotations per second
                     × wheel circumference
                     ÷ drive gear ratio

wheel distance = motor rotations
                 × wheel circumference
                 ÷ drive gear ratio
```

The CANcoder absolute position is corrected by subtracting the module's encoder offset. The corrected value is wrapped into [-0.5, 0.5) rotations before being stored as `steerPositionRotations`.

The method also records applied voltage, current, steering output, and status flags.

**Implementation note:** The current source has used `steerConnected = true` as a placeholder rather than deriving it from a REV connectivity diagnostic. Treat that flag as optimistic until real connection/fault reporting is added. Confirm the exact source implementation before relying on this status for diagnostics.

## `setDriveVelocity(...)`

The requested wheel speed in meters per second is converted into motor rotations per second:

1. Divide wheel linear speed by wheel circumference to get wheel rotations per second.
2. Multiply by the drive gear ratio to get motor rotations per second.
3. Set the Phoenix 6 `VelocityVoltage` request.
4. Send the request to the TalonFX with `setControl()`.

This assumes the gear ratio and units match the actual drivetrain and the Phoenix 6 velocity signal/request units.

## `setSteerOutput(...)`

Passes the normalized output directly to the SparkMax using `set()`.

## `stop()`

Calls `stopMotor()` on both the drive TalonFX and steering SparkMax.

## Hardware validation

Before connecting this implementation to a real robot, verify:
- All CAN IDs
- Motor types and wiring
- Motor inversion and sensor phase
- Gear ratios and wheel radius
- Encoder offsets and encoder direction
- Phoenix 6 and REV configuration API behavior for the installed library versions
- Drive velocity units and PID tuning
- Steering output direction and safe output limits

Do not treat successful compilation as proof that hardware configuration is correct.
