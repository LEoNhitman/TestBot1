package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOReal implements GyroIO {

    private final Pigeon2 pigeon;

    public GyroIOReal(int pigeonId) {
        pigeon = new Pigeon2(pigeonId);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        var yaw = pigeon.getYaw().refresh();
        inputs.yawDegrees = yaw.getValueAsDouble();
        inputs.connected = yaw.getStatus().isOK();
    }

}
