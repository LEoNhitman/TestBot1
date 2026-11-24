
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.SwerveConstants;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

public class SwerveDriveCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;

  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;
  private final DoubleSupplier fieldRelativeSupplier;

  public SwerveDriveCommand(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier fieldRelativeSupplier) {

    this.swerveSubsystem = swerveSubsystem;

    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {

    double forward = -forwardSupplier.getAsDouble();
    double strafe = -strafeSupplier.getAsDouble();
    double rotation = -rotationSupplier.getAsDouble();

    forward = MathUtil.applyDeadband(
        forward, SwerveConstants.DEADBAND);

    strafe = MathUtil.applyDeadband(
        strafe, SwerveConstants.DEADBAND);

    rotation = MathUtil.applyDeadband(
        rotation, SwerveConstants.DEADBAND);

    forward *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    strafe *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    rotation *= SwerveConstants.MAX_ANGULAR_SPEED;

    boolean fieldRelative = fieldRelativeSupplier.getAsDouble() > 0.5;

    swerveSubsystem.drive(
        forward,
        strafe,
        rotation,
        fieldRelative);
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
