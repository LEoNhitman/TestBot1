
package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveSubsystem extends SubsystemBase {

        private final SwerveModule frontLeft;
        private final SwerveModule frontRight;
        private final SwerveModule backLeft;
        private final SwerveModule backRight;

        private final GyroIO gyroIO;
        private final GyroIO.GyroIOInputs gyroInputs = new GyroIO.GyroIOInputs();

        // Reference to the simulated gyro.
        private GyroIOSim gyroSim;

        // Determines the position of each module relative to the robot center.
        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(

                        new Translation2d(
                                        Units.inchesToMeters(9.375),
                                        Units.inchesToMeters(12.875)),

                        new Translation2d(
                                        Units.inchesToMeters(9.375),
                                        Units.inchesToMeters(-12.875)),

                        new Translation2d(
                                        Units.inchesToMeters(-9.375),
                                        Units.inchesToMeters(12.875)),

                        new Translation2d(
                                        Units.inchesToMeters(-9.375),
                                        Units.inchesToMeters(-12.875)));

        public SwerveSubsystem() {

                boolean useSimulation = edu.wpi.first.wpilibj.RobotBase.isSimulation();

                if (useSimulation) {

                        frontLeft = new SwerveModule(new SwerveModuleIOSim(), 0);
                        frontRight = new SwerveModule(new SwerveModuleIOSim(), 1);
                        backLeft = new SwerveModule(new SwerveModuleIOSim(), 2);
                        backRight = new SwerveModule(new SwerveModuleIOSim(), 3);

                        gyroSim = new GyroIOSim();
                        gyroIO = gyroSim;

                } else {

                        frontLeft = new SwerveModule(
                                        new SwerveModuleIOReal(
                                                        SwerveConstants.FRONT_LEFT_DRIVE_ID,
                                                        SwerveConstants.FRONT_LEFT_STEER_ID,
                                                        SwerveConstants.FRONT_LEFT_CANCODER_ID,
                                                        SwerveConstants.FRONT_LEFT_OFFSET), 0);

                        frontRight = new SwerveModule(
                                        new SwerveModuleIOReal(
                                                        SwerveConstants.FRONT_RIGHT_DRIVE_ID,
                                                        SwerveConstants.FRONT_RIGHT_STEER_ID,
                                                        SwerveConstants.FRONT_RIGHT_CANCODER_ID,
                                                        SwerveConstants.FRONT_RIGHT_OFFSET), 1);

                        backLeft = new SwerveModule(
                                        new SwerveModuleIOReal(
                                                        SwerveConstants.BACK_LEFT_DRIVE_ID,
                                                        SwerveConstants.BACK_LEFT_STEER_ID,
                                                        SwerveConstants.BACK_LEFT_CANCODER_ID,
                                                        SwerveConstants.BACK_LEFT_OFFSET), 2);

                        backRight = new SwerveModule(
                                        new SwerveModuleIOReal(
                                                        SwerveConstants.BACK_RIGHT_DRIVE_ID,
                                                        SwerveConstants.BACK_RIGHT_STEER_ID,
                                                        SwerveConstants.BACK_RIGHT_CANCODER_ID,
                                                        SwerveConstants.BACK_RIGHT_OFFSET), 3);

                        gyroIO = new GyroIOReal(SwerveConstants.PIGEON_ID);
                }
        }

        public void calculateModuleStates(
                        double xSpeed,
                        double ySpeed,
                        double rotationSpeed) {

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
                                        states[i].speedMetersPerSecond);

                        SmartDashboard.putNumber(
                                        names[i] + " Desired Angle",
                                        states[i].angle.getDegrees());
                }
        }

        public void setAllModuleSpeed(double speedMetersPerSecond) {

                frontLeft.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
                frontRight.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
                backLeft.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
                backRight.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
        }

        public void drive(
                        double forward,
                        double strafe,
                        double rotation,
                        boolean fieldRelative) {

                ChassisSpeeds chassisSpeeds;

                if (fieldRelative) {

                        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                        forward,
                                        strafe,
                                        rotation,
                                        getHeading());

                } else {

                        chassisSpeeds = new ChassisSpeeds(forward, strafe, rotation);
                }

                // Update simulated gyro rotation.
                if (gyroSim != null) {
                        gyroSim.setAngularVelocity(
                                        chassisSpeeds.omegaRadiansPerSecond);
                }

                SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

                SwerveDriveKinematics.desaturateWheelSpeeds(
                                states,
                                SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

                frontLeft.setDesiredState(states[0]);
                frontRight.setDesiredState(states[1]);
                backLeft.setDesiredState(states[2]);
                backRight.setDesiredState(states[3]);
        }

        public void crabwalk(
                        double forward,
                        double strafe,
                        double rotation) {

                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forward, strafe, rotation);

                if (gyroSim != null) {
                        gyroSim.setAngularVelocity(
                                        chassisSpeeds.omegaRadiansPerSecond);
                }

                SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

                SwerveDriveKinematics.desaturateWheelSpeeds(
                                states,
                                SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

                frontLeft.setDesiredState(states[0]);
                frontRight.setDesiredState(states[1]);
                backLeft.setDesiredState(states[2]);
                backRight.setDesiredState(states[3]);
        }

        public void stopModules() {

                frontLeft.stop();
                frontRight.stop();
                backLeft.stop();
                backRight.stop();

                if (gyroSim != null) {
                        gyroSim.setAngularVelocity(0.0);
                }
        }

        public void testKinematics() {
                calculateModuleStates(1.0, 0.0, 0.0);
        }

        public Rotation2d getHeading() {

                if (!gyroInputs.connected) {
                        return new Rotation2d();
                }

                return Rotation2d.fromDegrees(gyroInputs.yawDegrees);
        }

        public void resetGyro() {
                gyroIO.reset();
                gyroIO.updateInputs(gyroInputs);
        }

        @Override
        public void periodic() {

                frontLeft.updateInputs();
                frontRight.updateInputs();
                backLeft.updateInputs();
                backRight.updateInputs();

                gyroIO.updateInputs(gyroInputs);

                SmartDashboard.putNumber(
                                "Robot Heading",
                                getHeading().getDegrees());

                SmartDashboard.putBoolean(
                                "Gyro Connected",
                                gyroInputs.connected);

                frontLeft.updateDashboard("Front Left");
                frontRight.updateDashboard("Front Right");
                backLeft.updateDashboard("Back Left");
                backRight.updateDashboard("Back Right");
        }
}
