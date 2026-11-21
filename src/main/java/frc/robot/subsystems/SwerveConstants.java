package frc.robot.subsystems;

public class SwerveConstants {
// Front left module
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_LEFT_STEER_ID = 10;
    public static final int FRONT_LEFT_CANCODER_ID = 20;
// Front right module
    public static final int FRONT_RIGHT_DRIVE_ID = 1;
    public static final int FRONT_RIGHT_STEER_ID = 11;
    public static final int FRONT_RIGHT_CANCODER_ID = 21;
// Back left module
    public static final int BACK_LEFT_DRIVE_ID = 3;
    public static final int BACK_LEFT_STEER_ID = 13;
    public static final int BACK_LEFT_CANCODER_ID = 23;
// Back right module
    public static final int BACK_RIGHT_DRIVE_ID = 2;
    public static final int BACK_RIGHT_STEER_ID = 12;
    public static final int BACK_RIGHT_CANCODER_ID = 22;
    
// gyro
    public static final int PIGEON_ID = 40;
    
// Drivetrain configuration
    public static final double DRIVE_GEAR_RATIO = 5.2734375;
// Steering configuration
    public static final double STEER_GEAR_RATIO = 150.0/7.0;
// Add wheel diameter
    public static final double WHEEL_RADIUS_METERS = 0.0508;
// ToDo: Add robot dimensions
// ToDo: Add maximum speed
// Offsets are 0.0 becuz i didnt have a robot to calibrate    
    public static final double FRONT_LEFT_OFFSET = 0.0;
    public static final double FRONT_RIGHT_OFFSET = 0.0;
    public static final double BACK_LEFT_OFFSET = 0.0;
    public static final double BACK_RIGHT_OFFSET = 0.0;
    public static final double DEADBAND = 0.08;
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.85;
    public static final double MAX_ANGULAR_SPEED = 2.0*Math.PI;
    public static final double PRECISION_SPEED_MULTIPLIER = 0.35;
 private SwerveConstants(){
    //prevents me from creating another instance of this class
 }
}
