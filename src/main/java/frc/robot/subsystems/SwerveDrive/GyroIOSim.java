package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {

    private double yawDegrees = 0.0;
    private double angularVelocityRadiansPerSecond = 0.0;

    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        double currentTimestamp = Timer.getFPGATimestamp();
        double deltaTime = currentTimestamp - lastTimestamp;

        lastTimestamp = currentTimestamp;

        // Integrate angular velocity to calculate the new heading.
        yawDegrees += Math.toDegrees(
                angularVelocityRadiansPerSecond * deltaTime);

        // Keep the heading between -180 and +180 degrees.
        yawDegrees = ((yawDegrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

        inputs.yawDegrees = yawDegrees;
        inputs.connected = true;
    }

    public void setAngularVelocity(double radiansPerSecond) {
        angularVelocityRadiansPerSecond = radiansPerSecond;
    }

    @Override
    public void reset() {
        yawDegrees = 0.0;
    }
}