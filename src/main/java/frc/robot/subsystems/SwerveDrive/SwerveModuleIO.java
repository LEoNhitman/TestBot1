package frc.robot.subsystems.SwerveDrive;

public interface SwerveModuleIO {
    // Stores information received from the motors and encoder.
    class SwerveModuleIOInputs{
        // Current drive wheel velocity in meters per second
        public double driveVelocityMetersPerSecond;
         // Total distance traveled by the drive wheel
        public double drivePositionMeters;
        // Current steering angle in rotations
        public double steerPositionRotations;
        // Voltage currently applied to the drive motor
        public double driveAppliedVolts;
        // Steering motor output
        public double steerAppliedOutput;
        // Current drawn by each motor
        public double driveCurrentAmps;
        public double steerCurrentAmps;
        // Indicates whether each device is communicating
        public boolean driveConnected;
        public boolean steerConnected;
        public boolean encoderConnected;
    }
    // Updates the input values using the latest hardware or simulation data
    default void updateInputs(SwerveModuleIOInputs inputs) {}
    // Commands the drive motor to reach a target wheel velocity
    default void setDriveVelocity(double metersPerSecond) {}
    // Sets the steering motor output
    default void setSteerOutput(double output) {}
    // Stops the module's motors
    default void stop() {}
}
