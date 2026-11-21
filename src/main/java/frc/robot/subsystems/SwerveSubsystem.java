// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;
  private final Pigeon2 gyro = new Pigeon2(23);

  private final SwerveDriveKinematics kinematics = 
  new SwerveDriveKinematics(
    new Translation2d(Units.inchesToMeters(9.375), Units.inchesToMeters(12.875)),
    new Translation2d(Units.inchesToMeters(9.375), Units.inchesToMeters(-12.875)),
    new Translation2d(Units.inchesToMeters(-9.375), Units.inchesToMeters(12.875)),
    new Translation2d(Units.inchesToMeters(-9.375), Units.inchesToMeters(-12.875))
  );
  public void calculateModuleStates(
    double xSpeed,
    double ySpeed,
    double rotationSpeed
  ){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

    String[] names = {
      "Front Left",
      "Front Right",
      "Back Left",
      "Back Right"
    };
      for (int i = 0; i < states.length; i++) {
        SmartDashboard.putNumber(
          names[i] + " Desired Speed",
          states[i].speedMetersPerSecond
        );

        SmartDashboard.putNumber(
          names[i] + " Desired Angle",
          states[i].angle.getDegrees()
        );
      }
  }
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    frontLeft = new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE_ID,SwerveConstants.FRONT_LEFT_STEER_ID,SwerveConstants.FRONT_LEFT_CANCODER_ID,SwerveConstants.FRONT_LEFT_OFFSET);
    frontRight = new SwerveModule(SwerveConstants.FRONT_RIGHT_DRIVE_ID,SwerveConstants.FRONT_RIGHT_STEER_ID,SwerveConstants.FRONT_RIGHT_CANCODER_ID,SwerveConstants.FRONT_RIGHT_OFFSET);
    backLeft = new SwerveModule(SwerveConstants.BACK_LEFT_DRIVE_ID,SwerveConstants.BACK_LEFT_STEER_ID,SwerveConstants.BACK_LEFT_CANCODER_ID,SwerveConstants.BACK_LEFT_OFFSET);
    backRight = new SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE_ID,SwerveConstants.BACK_RIGHT_STEER_ID,SwerveConstants.BACK_RIGHT_CANCODER_ID,SwerveConstants.BACK_RIGHT_OFFSET);
  }

  public void setAllModuleSpeed(double speedMetersPerSecond){
    frontLeft.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
    frontRight.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
    backLeft.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
    backRight.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
  }

  public void drive(
    double forward,
    double strafe,
    double rotation,
    boolean fieldRelative
  ){
    ChassisSpeeds chassisSpeeds;
    if (fieldRelative){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading());
    } else{
      chassisSpeeds = new ChassisSpeeds(forward,strafe,rotation);
    }
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }
  public void crabwalk(double forward, double strafe, double rotation){
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forward, strafe, rotation);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }
  public void stopModules() {
    frontLeft.setDriveSpeedMetersPerSecond(0.0);
    frontRight.setDriveSpeedMetersPerSecond(0.0);
    backLeft.setDriveSpeedMetersPerSecond(0.0);
    backRight.setDriveSpeedMetersPerSecond(0.0);
  }

  public void testKinematics() {
    // Drive forward at 1 m/s
    calculateModuleStates(1.0, 0.0, 0.0);
  }
  //retrieves the robot's current heading in degrees and converts it into a WPILib rotation object
  public Rotation2d getHeading() {
    var yaw = gyro.getYaw().refresh();

    if (!yaw.getStatus().isOK()) {
        return new Rotation2d();
    }

    return Rotation2d.fromDegrees(yaw.getValueAsDouble());
  }
  public void resetGyro() {
    gyro.setYaw(0.0);
  }
@Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
    frontLeft.updateDashboard("Front Left");
    frontRight.updateDashboard("Front Right");
    backRight.updateDashboard("Back Right");
    backLeft.updateDashboard("Back Left");
}

}
