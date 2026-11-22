package frc.robot.subsystems.SwerveDrive;

public interface GyroIO {
    class GyroIOInputs {
        public double yawDegrees = 0.0;
        public boolean connected = false;
    }

    default void updateInputs(GyroIOInputs inputs) {}

    default void reset() {}
}
