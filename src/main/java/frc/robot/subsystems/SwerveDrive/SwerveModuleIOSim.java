
package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveDrive.SwerveModuleIO.SwerveModuleIOInputs;
import edu.wpi.first.math.MathUtil;

public class SwerveModuleIOSim implements SwerveModuleIO {

    // Simulated drive motor velocity in meters per second.
    // This represents how quickly the wheel is moving.
    private double driveVelocityMetersPerSecond;

    // Simulated total distance traveled by the drive wheel in meters.
    private double drivePositionMeters;

    // Simulated steering angle in rotations.
    // 0.0 = forward, 0.25 = 90 degrees, -0.25 = -90 degrees.
    private double steerPositionRotations;

    // Simulated steering motor output.
    // Values range from -1.0 to 1.0.
    private double steerOutput;

    // Stores the timestamp of the previous simulation update.
    // We use this to calculate how much time has passed.
    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {

        // Get the current FPGA timestamp.
        double currentTimestamp = Timer.getFPGATimestamp();

        // Calculate the time elapsed since the previous update.
        // This is important because simulation updates may not happen
        // at exactly the same time interval.
        double deltaTime = currentTimestamp - lastTimestamp;

        // Save the current timestamp for the next update.
        lastTimestamp = currentTimestamp;

        // DRIVE MOTOR SIMULATION

        // Distance = velocity * time.
        // This updates the total distance traveled by the wheel.
        drivePositionMeters += driveVelocityMetersPerSecond * deltaTime;

        // STEERING MOTOR SIMULATION

        // Simulate the steering motor changing the module's angle.
        // The 0.5 value represents an arbitrary steering response speed.
        // This is a simplified approximation, not a physical motor model.
        steerPositionRotations += steerOutput * 0.5 * deltaTime;

        // Keep the steering angle between -0.5 and +0.5 rotations.
        // This prevents the angle from continuously increasing or decreasing.
        steerPositionRotations = ((steerPositionRotations + 0.5) % 1.0 + 1.0) % 1.0 - 0.5;

        // COPY SIMULATED VALUES INTO THE INPUTS OBJECT

        // These values will be read by SwerveModule and SwerveSubsystem.
        inputs.driveVelocityMetersPerSecond = driveVelocityMetersPerSecond;
        inputs.drivePositionMeters = drivePositionMeters;
        inputs.steerPositionRotations = steerPositionRotations;

        // Simulation does not currently model electrical behavior.
        inputs.driveAppliedVolts = 0.0;
        inputs.steerAppliedOutput = steerOutput;

        inputs.driveCurrentAmps = 0.0;
        inputs.steerCurrentAmps = 0.0;

        // Report all simulated devices as connected.
        // These are simulated devices, so there is no real hardware connection.
        inputs.driveConnected = true;
        inputs.steerConnected = true;
        inputs.encoderConnected = true;
    }

    @Override
    public void setDriveVelocity(double metersPerSecond) {

        // Store the requested drive velocity.
        // The next updateInputs() call will use this value to update position.
        driveVelocityMetersPerSecond = metersPerSecond;
    }

    @Override
    public void setSteerOutput(double output) {

        // Limit steering output to the valid motor output range.
        // MathUtil.clamp prevents values below -1.0 or above 1.0.
        steerOutput = MathUtil.clamp(output, -1.0, 1.0);
    }

    @Override
    public void stop() {

        // Set drive velocity to zero so the wheel stops moving.
        driveVelocityMetersPerSecond = 0.0;

        // Set steering output to zero so the module stops turning.
        steerOutput = 0.0;
    }
}
