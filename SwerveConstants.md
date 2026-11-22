# `SwerveConstants.java`

**Package:** `frc.robot.subsystems.SwerveDrive`  
**Role:** Central location for drivetrain configuration values.

## Responsibility

This class groups values used by the swerve subsystem, modules, and controller mapping. Centralizing constants avoids scattering hardware IDs, geometry, ratios, and speed limits throughout the code.

The class has a private constructor so it is used as a static constants holder rather than instantiated.

## Hardware identifiers

The class defines CAN IDs for:
- Front-left drive motor, steering motor, and CANcoder
- Front-right drive motor, steering motor, and CANcoder
- Back-left drive motor, steering motor, and CANcoder
- Back-right drive motor, steering motor, and CANcoder
- Pigeon 2 gyro

The real IO implementations use these IDs when constructing vendor hardware objects.

**Verification required:** The IDs currently in the source are provisional project values. They must be checked against the team's actual CAN wiring and current robot configuration. Do not assume they match the physical robot.

## Mechanical constants

- `DRIVE_GEAR_RATIO`: drive motor rotations per wheel rotation, as assumed by the conversion code.
- `STEER_GEAR_RATIO`: steering reduction ratio. It must match the actual module.
- `WHEEL_RADIUS_METERS`: wheel radius used to convert motor rotations into wheel distance and speed.
- Per-module encoder offsets: correction values used to align absolute encoder readings with the module's chosen zero angle.

The conversion formulas are only correct if the ratios and wheel radius correspond to the actual mechanism and the sensor units used by the vendor API.

## Driver and speed constants

- `DEADBAND`: threshold applied to controller axes to suppress small unintended inputs.
- `MAX_SPEED_METERS_PER_SECOND`: maximum wheel speed used for speed scaling and wheel-speed desaturation.
- `MAX_ANGULAR_SPEED`: maximum angular speed used by the controller mapping.
- `PRECISION_SPEED_MULTIPLIER`: scale applied while the precision-mode button is held.

## Module geometry

The subsystem currently constructs its `SwerveDriveKinematics` using module translations of ±9.375 inches in X and ±12.875 inches in Y, converted to meters.

These dimensions define the wheel locations relative to the robot center. Incorrect geometry affects how chassis translation and rotation are distributed among modules.

## Before hardware operation

Confirm every ID, encoder offset, gear ratio, wheel radius, module location, motor inversion, and speed limit against the current robot. A successful build does not validate hardware configuration.
