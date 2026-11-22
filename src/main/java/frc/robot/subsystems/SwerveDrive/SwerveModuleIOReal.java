package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.SwerveDrive.SwerveModuleIO.SwerveModuleIOInputs;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class SwerveModuleIOReal implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder absoluteEncoder;

    private final double encoderOffset;

    private final VelocityVoltage driveRequest = new VelocityVoltage(0);

    public SwerveModuleIOReal(int driveMotorID, int steerMotorID, int encoderId, double encoderOffset) {
        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        this.encoderOffset = encoderOffset;

        // Initialize and configure the drive motor

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0.kP = 0.01;
        driveConfig.Slot0.kI = 0.0;
        driveConfig.Slot0.kD = 0.0;
        driveConfig.Slot0.kV = 0.0;
        driveMotor.getConfigurator().apply(driveConfig);

        // Initialize and configure the steering motor

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.idleMode(IdleMode.kBrake);

        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize and configure the absolute encoder

        absoluteEncoder = new CANcoder(encoderId);
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        absoluteEncoder.getConfigurator().apply(encoderConfig);
        absoluteEncoder.getAbsolutePosition().setUpdateFrequency(50);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {

        // Read drive motor information.
        var driveVelocity = driveMotor.getVelocity().refresh();
        var drivePosition = driveMotor.getPosition().refresh();
        var driveVoltage = driveMotor.getMotorVoltage().refresh();
        var driveCurrent = driveMotor.getStatorCurrent().refresh();

        inputs.driveVelocityMetersPerSecond = driveVelocity.getValueAsDouble()
                * (2.0 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS)
                / SwerveConstants.DRIVE_GEAR_RATIO;

        inputs.drivePositionMeters = drivePosition.getValueAsDouble()
                * (2.0 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS)
                / SwerveConstants.DRIVE_GEAR_RATIO;

        inputs.driveAppliedVolts = driveVoltage.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        inputs.driveConnected = driveVelocity.getStatus().isOK()
                && drivePosition.getStatus().isOK();

        // Read steering motor information.

        inputs.steerAppliedOutput = steerMotor.getAppliedOutput();
        inputs.steerCurrentAmps = steerMotor.getOutputCurrent();
        inputs.steerConnected = true;

        // Read and correct the absolute encoder position.

        var encoderPosition = absoluteEncoder.getAbsolutePosition().refresh();

        inputs.encoderConnected = encoderPosition.getStatus().isOK();

        if (inputs.encoderConnected) {
            double correctedPosition = encoderPosition.getValueAsDouble() - encoderOffset;

            inputs.steerPositionRotations = ((correctedPosition + 0.5) % 1.0 + 1.0) % 1.0 - 0.5;
        }
    }

    @Override
    public void setDriveVelocity(double metersPerSecond) {

        // Convert wheel speed from meters per second to motor rotations per second

        double wheelRotationsPerSecond = metersPerSecond
                / (2.0 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS);

        double motorRotationsPerSecond = wheelRotationsPerSecond * SwerveConstants.DRIVE_GEAR_RATIO;

        driveRequest.withVelocity(motorRotationsPerSecond);

        driveMotor.setControl(driveRequest);

    }

    @Override
    public void setSteerOutput(double output) {
        steerMotor.set(output);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

}
