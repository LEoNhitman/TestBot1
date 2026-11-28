
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveDrive.SwerveConstants;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

public class SwerveDriveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;

    private final BooleanSupplier fieldRelativeSupplier;
    private final BooleanSupplier precisionSupplier;
    private final IntSupplier povSupplier;

    public SwerveDriveCommand(
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier fieldRelativeSupplier,
            BooleanSupplier precisionSupplier,
            IntSupplier povSupplier) {

        this.swerveSubsystem = swerveSubsystem;

        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.precisionSupplier = precisionSupplier;
        this.povSupplier = povSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        // Read controller inputs.
        double forward = -forwardSupplier.getAsDouble();
        double strafe = -strafeSupplier.getAsDouble();
        double rotation = -rotationSupplier.getAsDouble();

        // Apply deadband to eliminate small unwanted inputs.
        forward = MathUtil.applyDeadband(
                forward, SwerveConstants.DEADBAND);

        strafe = MathUtil.applyDeadband(
                strafe, SwerveConstants.DEADBAND);

        rotation = MathUtil.applyDeadband(
                rotation, SwerveConstants.DEADBAND);

        // Apply precision mode speed reduction.
        double speedMultiplier = precisionSupplier.getAsBoolean()
                ? SwerveConstants.PRECISION_SPEED_MULTIPLIER
                : 1.0;

        forward *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                * speedMultiplier;

        strafe *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND
                * speedMultiplier;

        rotation *= SwerveConstants.MAX_ANGULAR_SPEED
                * speedMultiplier;

        // Check whether the D-pad is being pressed.
        int pov = povSupplier.getAsInt();

        if (pov != -1) {

            // Convert D-pad angle into robot-relative movement.
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

            // Normal field-relative or robot-relative driving.
            swerveSubsystem.drive(
                    forward,
                    strafe,
                    rotation,
                    fieldRelativeSupplier.getAsBoolean());
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
