# `RobotContainer.java`

**Package:** `frc.robot`  
**Role:** Constructs robot controls and connects driver input to commands and subsystems.

## Responsibility

`RobotContainer` is the composition point for the robot's operator interface and command-based behavior. It creates:
- One `SwerveSubsystem`
- One `CommandXboxController` on USB port `0`

It then configures the drivetrain's default command and button bindings.

## Default drivetrain command

The constructor installs a default command using `Commands.runEnd(...)`.

- The execute action calls `driveWithController()`.
- The end action calls `swerveSubsystem.stopModules()`.
- The command requires `swerveSubsystem`.

Because it is the subsystem's default command, it runs whenever no other scheduled command is using the drivetrain.

## Controller reset binding

The start button is bound with `onTrue()` to a `Commands.runOnce(...)` command that calls `swerveSubsystem.resetGyro()`.

This means the gyro reset action is triggered when the start button transitions from not pressed to pressed.

## Back button binding

The back button is also bound with `onTrue()` to a one-shot command that calls `stopModules()`.

This is a one-time stop request, not a latched emergency stop. Since the default command remains available, it can resume driving on a subsequent scheduler iteration if controller inputs request motion. It must not be treated as a safety-rated emergency-stop mechanism.

## `driveWithController()`

This method reads the controller axes:
- Left Y controls forward/backward movement.
- Left X controls strafing.
- Right X controls rotation.

The axis values are negated to match the intended driver direction convention.

Each axis is passed through `MathUtil.applyDeadband()` using `SwerveConstants.DEADBAND`. This prevents small joystick noise near the center from causing unwanted movement.

The values are then scaled:
- Forward and strafe use `MAX_SPEED_METERS_PER_SECOND`.
- Rotation uses `MAX_ANGULAR_SPEED`.
- Holding X applies `PRECISION_SPEED_MULTIPLIER` to all three.

## D-pad crabwalk

The method reads the controller POV angle. If the POV is not `-1`, the D-pad is being pressed.

The POV angle is converted to radians. Cosine and sine are used to calculate a forward and strafe command for the selected direction. These values are passed to `swerveSubsystem.crabwalk()`.

If the D-pad is not pressed, the method uses normal `drive()`.

## Field-relative toggle

The B button determines the coordinate mode:
- B not pressed: `fieldRelative` is true.
- B pressed: `fieldRelative` is false, so the drivetrain uses robot-relative motion.

Field-relative driving depends on a valid gyro heading.

## Autonomous command

`getAutonomousCommand()` currently returns a print command stating that no autonomous command is configured. This is a placeholder, not a functional autonomous routine.

## Important notes

- The controller is configured for USB port `0`.
- A controller is not required to compile the project or launch simulation.
- The temporary gyro test discussed during development should not be added to permanent bindings unless intentionally retained and documented.
- The back-button stop binding is not an emergency stop.
