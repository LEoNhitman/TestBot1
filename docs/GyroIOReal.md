# `GyroIOReal.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Implements:** `GyroIO`  
**Role:** Reads yaw from a physical CTRE Pigeon 2.

## Construction

The constructor receives the Pigeon CAN ID and creates a `Pigeon2` object with that ID.

The ID is passed from `SwerveConstants.PIGEON_ID` by `SwerveSubsystem` when the real-hardware branch is selected.

## `updateInputs(...)`

The method refreshes the Pigeon 2 yaw status signal:

1. Calls `pigeon.getYaw().refresh()`.
2. Reads the numeric yaw using `getValueAsDouble()`.
3. Checks the signal status using `getStatus().isOK()`.
4. Stores the yaw in degrees and the status in the shared input structure.

The subsystem later converts the yaw degrees into a WPILib `Rotation2d`.

## `reset()`

Calls `pigeon.setYaw(0.0)` to set the Pigeon 2 yaw reference to zero degrees.

## Important notes

- Confirm the Pigeon CAN ID against the actual robot.
- Confirm that the Pigeon is mounted in the expected orientation and that its positive yaw direction matches the drivetrain coordinate convention.
- The current code checks the yaw signal status; this is a basic connectivity/validity indicator, not a complete gyro health diagnostic.
