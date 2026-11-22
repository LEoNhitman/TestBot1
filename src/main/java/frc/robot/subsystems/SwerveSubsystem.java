
package frc.robot.subsystems;

// SubsystemBase is the WPILib class that allows this class
// to function as a command-based subsystem.
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Translation2d represents a position using X and Y coordinates.
import edu.wpi.first.math.geometry.Translation2d;

// SwerveDriveKinematics calculates the speed and angle
// required for each swerve module.
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

// Units provides conversion methods, such as inches to meters.
import edu.wpi.first.math.util.Units;

// ChassisSpeeds represents the robot's overall movement:
// forward speed, sideways speed, and rotational speed.
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// SwerveModuleState stores the desired speed and angle of one module.
import edu.wpi.first.math.kinematics.SwerveModuleState;

// SmartDashboard allows us to send information to the dashboard
// in the WPILib simulation and Driver Station.
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Pigeon2 is the CTRE gyroscope used to measure robot heading.
import com.ctre.phoenix6.hardware.Pigeon2;

// Rotation2d represents an angle in a form understood by WPILib.
import edu.wpi.first.math.geometry.Rotation2d;

// SwerveSubsystem coordinates all four swerve modules.
//
// It receives movement requests, calculates the required
// module states, and sends those states to each SwerveModule.
public class SwerveSubsystem extends SubsystemBase {

    // References to the four individual swerve modules.
    // Each module has its own drive motor, steering motor,
    // and absolute encoder.
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    // Creates the Pigeon2 gyroscope using CAN ID 23.
    // This ID must match the actual robot's hardware configuration.
    private final Pigeon2 gyro = new Pigeon2(23);

    // Creates the kinematics object.
    //
    // It needs to know where each wheel is located relative
    // to the center of the robot.
    //
    // WPILib coordinate convention:
    // +X = forward
    // +Y = left
    //
    // The order of these positions is important:
    // 0 = Front Left
    // 1 = Front Right
    // 2 = Back Left
    // 3 = Back Right
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(

            // Front Left module position.
            new Translation2d(
                    Units.inchesToMeters(9.375),
                    Units.inchesToMeters(12.875)),

            // Front Right module position.
            new Translation2d(
                    Units.inchesToMeters(9.375),
                    Units.inchesToMeters(-12.875)),

            // Back Left module position.
            new Translation2d(
                    Units.inchesToMeters(-9.375),
                    Units.inchesToMeters(12.875)),

            // Back Right module position.
            new Translation2d(
                    Units.inchesToMeters(-9.375),
                    Units.inchesToMeters(-12.875)));

    // Calculates what each swerve module should do
    // for a requested robot movement.
    //
    // IMPORTANT:
    // This method only calculates and displays the states.
    // It does NOT send commands to the motors.
    //
    // xSpeed = forward/backward speed in meters per second.
    // ySpeed = sideways speed in meters per second.
    // rotationSpeed = rotational speed in radians per second.
    public void calculateModuleStates(
            double xSpeed,
            double ySpeed,
            double rotationSpeed) {

        // Combines the three movement requests into one
        // ChassisSpeeds object.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

        // Converts the robot's requested movement into
        // four individual wheel speeds and angles.
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Names corresponding to the order of the modules
        // in the kinematics object.
        String[] names = {
                "Front Left",
                "Front Right",
                "Back Left",
                "Back Right"
        };

        // Loops through all four calculated module states.
        for (int i = 0; i < states.length; i++) {

            // Displays the desired wheel speed in meters per second.
            SmartDashboard.putNumber(
                    names[i] + " Desired Speed",
                    states[i].speedMetersPerSecond);

            // Displays the desired steering angle in degrees.
            SmartDashboard.putNumber(
                    names[i] + " Desired Angle",
                    states[i].angle.getDegrees());
        }
    }

    // CONSTRUCTOR
    //
    // Java automatically calls this when a new
    // SwerveSubsystem object is created.
    //
    // Its job is to initialize all four swerve modules.
    public SwerveSubsystem() {

        // Initialize the Front Left module.
        // The arguments come from SwerveConstants.
        frontLeft = new SwerveModule(
                SwerveConstants.FRONT_LEFT_DRIVE_ID,
                SwerveConstants.FRONT_LEFT_STEER_ID,
                SwerveConstants.FRONT_LEFT_CANCODER_ID,
                SwerveConstants.FRONT_LEFT_OFFSET);

        // Initialize the Front Right module.
        frontRight = new SwerveModule(
                SwerveConstants.FRONT_RIGHT_DRIVE_ID,
                SwerveConstants.FRONT_RIGHT_STEER_ID,
                SwerveConstants.FRONT_RIGHT_CANCODER_ID,
                SwerveConstants.FRONT_RIGHT_OFFSET);

        // Initialize the Back Left module.
        backLeft = new SwerveModule(
                SwerveConstants.BACK_LEFT_DRIVE_ID,
                SwerveConstants.BACK_LEFT_STEER_ID,
                SwerveConstants.BACK_LEFT_CANCODER_ID,
                SwerveConstants.BACK_LEFT_OFFSET);

        // Initialize the Back Right module.
        backRight = new SwerveModule(
                SwerveConstants.BACK_RIGHT_DRIVE_ID,
                SwerveConstants.BACK_RIGHT_STEER_ID,
                SwerveConstants.BACK_RIGHT_CANCODER_ID,
                SwerveConstants.BACK_RIGHT_OFFSET);
    }

    // Sends the same drive speed to all four modules.
    //
    // This method does not calculate steering angles.
    // It simply tells every module to use the same wheel speed.
    //
    // speedMetersPerSecond = requested wheel speed.
    public void setAllModuleSpeed(double speedMetersPerSecond) {

        frontLeft.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
        frontRight.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
        backLeft.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
        backRight.setDriveSpeedMetersPerSecond(speedMetersPerSecond);
    }

    // MAIN DRIVING METHOD
    //
    // Receives the desired movement of the robot
    // and converts it into individual module commands.
    //
    // forward = forward/backward speed in meters per second.
    // strafe = sideways speed in meters per second.
    // rotation = rotational speed in radians per second.
    // fieldRelative = determines the coordinate system.
    public void drive(
            double forward,
            double strafe,
            double rotation,
            boolean fieldRelative) {

        // This variable will hold the robot's requested movement.
        ChassisSpeeds chassisSpeeds;

        // FIELD-RELATIVE DRIVING
        //
        // Movement directions are based on the field,
        // rather than the robot's current orientation.
        if (fieldRelative) {

            // Converts field-relative movement requests
            // into robot-relative movement using gyro heading.
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward,
                    strafe,
                    rotation,
                    getHeading());

        } else {

            // ROBOT-RELATIVE DRIVING
            //
            // Movement directions are based on the robot itself.
            chassisSpeeds = new ChassisSpeeds(forward, strafe, rotation);
        }

        // Calculates the desired speed and angle for each wheel.
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        // SPEED DESATURATION
        //
        // If any wheel is requested to go faster than the
        // configured maximum, all wheel speeds are scaled down
        // proportionally to preserve the requested movement direction.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                states,
                SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

        // Send the calculated state to each individual module.
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    // CRABWALK
    //
    // Allows the robot to translate in a direction while
    // keeping the movement robot-relative.
    //
    // It uses the same kinematics calculations as drive(),
    // but does not perform field-relative conversion.
    public void crabwalk(
            double forward,
            double strafe,
            double rotation) {

        // Creates a robot-relative movement request.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forward, strafe, rotation);

        // Calculates the four desired module states.
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Makes sure no wheel exceeds the configured maximum speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                states,
                SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

        // Sends each module its calculated state.
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    // STOP MODULES
    //
    // Commands all four drive motors to zero wheel speed.
    //
    // This does not stop the steering motors or act as
    // a hardware emergency-stop system.
    public void stopModules() {

        frontLeft.setDriveSpeedMetersPerSecond(0.0);
        frontRight.setDriveSpeedMetersPerSecond(0.0);
        backLeft.setDriveSpeedMetersPerSecond(0.0);
        backRight.setDriveSpeedMetersPerSecond(0.0);
    }

    // TEST KINEMATICS
    //
    // Requests a simulated calculation for forward movement
    // at 1 meter per second.
    //
    // The results are displayed on SmartDashboard.
    // No motors are commanded by this method.
    public void testKinematics() {

        calculateModuleStates(1.0, 0.0, 0.0);
    }

    // GET HEADING
    //
    // Reads the Pigeon2's current yaw angle.
    //
    // Yaw represents the robot's rotation around its vertical axis.
    // The result is returned as a Rotation2d object.
    public Rotation2d getHeading() {

        // Refresh requests the latest gyro yaw reading.
        var yaw = gyro.getYaw().refresh();

        // Check whether the sensor reading was successful.
        if (!yaw.getStatus().isOK()) {

            // Temporary fallback: return zero degrees.
            // Real robot code should handle gyro faults explicitly.
            return new Rotation2d();
        }

        // Convert the gyro reading from degrees into Rotation2d.
        return Rotation2d.fromDegrees(yaw.getValueAsDouble());
    }

    // RESET GYRO
    //
    // Sets the Pigeon2 yaw angle to zero degrees.
    // This makes the current direction the new heading reference.
    public void resetGyro() {

        gyro.setYaw(0.0);
    }

    // PERIODIC
    //
    // WPILib automatically calls periodic() regularly
    // while the robot is enabled and the scheduler runs.
    //
    // It is useful for updating dashboard information
    // and monitoring subsystem status.
    @Override
    public void periodic() {

        // Display the current robot heading in degrees.
        SmartDashboard.putNumber(
                "Robot Heading",
                getHeading().getDegrees());

        // Display the current steering angle of each module.
        frontLeft.updateDashboard("Front Left");
        frontRight.updateDashboard("Front Right");
        backRight.updateDashboard("Back Right");
        backLeft.updateDashboard("Back Left");
    }
}
