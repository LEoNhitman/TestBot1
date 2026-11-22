package frc.robot.subsystems.SwerveDrive;

public class GyroIOSim implements GyroIO {

    private double yawDegrees = 0.0;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawDegrees = yawDegrees;
        inputs.connected = true;
    }

    @Override
    public void reset() {
        yawDegrees = 0.0;
    }
}