
package frc.robot.subsystems;

// SwerveConstants stores configuration values used throughout
// the drivetrain.
//
// Other classes, such as SwerveModule and SwerveSubsystem,
// access these values using SwerveConstants.CONSTANT_NAME.
//
// This prevents us from repeating configuration values
// throughout the project.
public class SwerveConstants {

    // --------------------------------------------------
    // FRONT LEFT MODULE
    // --------------------------------------------------

    // CAN ID of the front-left Kraken drive motor.
    public static final int FRONT_LEFT_DRIVE_ID = 0;

    // CAN ID of the front-left NEO steering motor.
    public static final int FRONT_LEFT_STEER_ID = 10;

    // CAN ID of the front-left CANcoder.
    public static final int FRONT_LEFT_CANCODER_ID = 20;

    // --------------------------------------------------
    // FRONT RIGHT MODULE
    // --------------------------------------------------

    // CAN ID of the front-right Kraken drive motor.
    public static final int FRONT_RIGHT_DRIVE_ID = 1;

    // CAN ID of the front-right NEO steering motor.
    public static final int FRONT_RIGHT_STEER_ID = 11;

    // CAN ID of the front-right CANcoder.
    public static final int FRONT_RIGHT_CANCODER_ID = 21;

    // --------------------------------------------------
    // BACK LEFT MODULE
    // --------------------------------------------------

    // CAN ID of the back-left Kraken drive motor.
    public static final int BACK_LEFT_DRIVE_ID = 3;

    // CAN ID of the back-left NEO steering motor.
    public static final int BACK_LEFT_STEER_ID = 13;

    // CAN ID of the back-left CANcoder.
    public static final int BACK_LEFT_CANCODER_ID = 23;

    // --------------------------------------------------
    // BACK RIGHT MODULE
    // --------------------------------------------------

    // CAN ID of the back-right Kraken drive motor.
    public static final int BACK_RIGHT_DRIVE_ID = 2;

    // CAN ID of the back-right NEO steering motor.
    public static final int BACK_RIGHT_STEER_ID = 12;

    // CAN ID of the back-right CANcoder.
    public static final int BACK_RIGHT_CANCODER_ID = 22;

    // --------------------------------------------------
    // GYROSCOPE
    // --------------------------------------------------

    // CAN ID of the Pigeon 2 gyroscope.
    //
    // The Pigeon measures the robot's orientation.
    // SwerveSubsystem uses its yaw reading for field-relative driving.
    public static final int PIGEON_ID = 40;

    // --------------------------------------------------
    // DRIVE CONFIGURATION
    // --------------------------------------------------

    // Drive gear ratio.
    //
    // This represents the number of motor rotations
    // required for one wheel rotation.
    //
    // SwerveModule uses this to convert wheel speed
    // into the motor velocity requested from the TalonFX.
    public static final double DRIVE_GEAR_RATIO = 5.2734375;

    // --------------------------------------------------
    // STEERING CONFIGURATION
    // --------------------------------------------------

    // Steering gear ratio.
    //
    // This represents the relationship between motor rotations
    // and steering output rotations.
    //
    // It is currently stored here for future steering calculations.
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    // --------------------------------------------------
    // WHEEL DIMENSIONS
    // --------------------------------------------------

    // Radius of the wheel in meters.
    //
    // 0.0508 meters = 2 inches.
    //
    // The radius is used to calculate wheel circumference:
    // circumference = 2 * PI * radius.
    //
    // SwerveModule uses the circumference to convert
    // linear wheel speed into wheel rotations per second.
    public static final double WHEEL_RADIUS_METERS = 0.0508;

    // --------------------------------------------------
    // MODULE OFFSETS
    // --------------------------------------------------

    // Encoder offsets correct the raw CANcoder readings.
    //
    // Each module has its own offset because its encoder
    // may have a different zero position.
    //
    // These are currently zero because the modules
    // have not yet been calibrated.
    //
    // IMPORTANT:
    // These are placeholders, not final competition values.
    // They must be calibrated and verified with the team.
    public static final double FRONT_LEFT_OFFSET = 0.0;
    public static final double FRONT_RIGHT_OFFSET = 0.0;
    public static final double BACK_LEFT_OFFSET = 0.0;
    public static final double BACK_RIGHT_OFFSET = 0.0;

    // --------------------------------------------------
    // CONTROLLER CONFIGURATION
    // --------------------------------------------------

    // Joystick deadband.
    //
    // Small controller inputs below this threshold
    // can be treated as zero to prevent unwanted movement.
    //
    // Example:
    // An input of 0.04 would be ignored if the deadband is 0.08.
    public static final double DEADBAND = 0.08;

    // --------------------------------------------------
    // DRIVETRAIN SPEED LIMITS
    // --------------------------------------------------

    // Maximum linear drivetrain speed in meters per second.
    //
    // Used by SwerveSubsystem to limit wheel speeds
    // when calculating and sending module states.
    //
    // This value must match the actual drivetrain's
    // verified maximum speed.
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.85;

    // Maximum angular speed in radians per second.
    //
    // 2 * PI radians per second = one complete rotation per second.
    //
    // This can be used to scale the driver's rotation input.
    public static final double MAX_ANGULAR_SPEED = 2.0 * Math.PI;

    // --------------------------------------------------
    // PRECISION MODE
    // --------------------------------------------------

    // Multiplier used to reduce drivetrain speed in precision mode.
    //
    // For example:
    // Normal speed * 0.35 = 35% of normal speed.
    //
    // This is useful when the driver needs more controlled movement.
    public static final double PRECISION_SPEED_MULTIPLIER = 0.35;

    // --------------------------------------------------
    // PRIVATE CONSTRUCTOR
    // --------------------------------------------------

    // This class only stores static constants.
    //
    // We do not need to create a SwerveConstants object.
    //
    // Making the constructor private prevents other classes
    // from accidentally creating an instance.
    private SwerveConstants() {
    }
}
