
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrive.SwerveConstants;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();

        swerveSubsystem.setDefaultCommand(
                Commands.runEnd(
                        this::driveWithController,
                        swerveSubsystem::stopModules,
                        swerveSubsystem));

        driverController.start().onTrue(
                Commands.runOnce(
                        swerveSubsystem::resetGyro,
                        swerveSubsystem));
    }

    private void configureBindings() {
        driverController.back().onTrue(
                Commands.runOnce(
                        swerveSubsystem::stopModules,
                        swerveSubsystem));

        driverController.y().whileTrue(
        Commands.run(
                () -> swerveSubsystem.drive(0.0, 0.0, 1.0, false),
                swerveSubsystem));
        
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void driveWithController() {
        double forward = -driverController.getLeftY();
        double strafe = -driverController.getLeftX();
        double rotation = -driverController.getRightX();

        forward = MathUtil.applyDeadband(
                forward, SwerveConstants.DEADBAND);

        strafe = MathUtil.applyDeadband(
                strafe, SwerveConstants.DEADBAND);

        rotation = MathUtil.applyDeadband(
                rotation, SwerveConstants.DEADBAND);

        double speedMultiplier = driverController.x().getAsBoolean()
                ? SwerveConstants.PRECISION_SPEED_MULTIPLIER
                : 1.0;

        forward *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                * speedMultiplier;

        strafe *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                * speedMultiplier;

        rotation *= SwerveConstants.MAX_ANGULAR_SPEED
                * speedMultiplier;

        int pov = driverController.getHID().getPOV();

        if (pov != -1) {
            double angle = Math.toRadians(pov);

            double crabForward = Math.cos(angle)
                    * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                    * speedMultiplier;

            double crabStrafe = -Math.sin(angle)
                    * SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                    * speedMultiplier;

            swerveSubsystem.crabwalk(
                    crabForward,
                    crabStrafe,
                    rotation);

        } else {
            boolean fieldRelative = !driverController.b().getAsBoolean();

            swerveSubsystem.drive(
                    forward,
                    strafe,
                    rotation,
                    fieldRelative);
        }
    }
}
