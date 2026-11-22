package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class SwerveModule {
    // Hardware
    // Drive motor: Kraken and TalonFX
    private final TalonFX driveMotor;
    // Steering motor: NEO and SPARKMAX
    private final SparkMax steerMotor;
    // Absolute encoder: CANCoder
    private final CANcoder absoluteEncoder;
    private final double encoderOffset;
    private final VelocityVoltage driveRequest = new VelocityVoltage(0);
    private final PIDController steerPID = new PIDController(0.01, 0.0, 0.0);

    // Constructor
    public SwerveModule(
            int driveMotorID,
            int steerMotorID,
            int encoderId,
            double encoderOffset) {
        // I gotta intialize hardware here
        driveMotor = new TalonFX(driveMotorID);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0.kP = 0.01;
        driveConfig.Slot0.kI = 0.0;
        driveConfig.Slot0.kD = 0.0;
        driveConfig.Slot0.kV = 0.0;
        driveMotor.getConfigurator().apply(driveConfig);
        steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.idleMode(IdleMode.kBrake);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        absoluteEncoder = new CANcoder(encoderId);
        this.encoderOffset = encoderOffset;
        configureCANcoder(); // Update at 50 Hz
        steerPID.enableContinuousInput(-0.5, 0.5); // The controller needs to understand that 0.49 rotations and -0.49
                                                   // rotations are close together, not far apart
    }

    public void setDesiredState(SwerveModuleState state) {
        // Get the current wheel angle
        double currentAngle = getSteeringAngleRadians();
        // Optimize desired state
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(currentAngle));
        double currentAngleRotations = getSteeringAngleRotations();
        double desiredAngleRotations = optimizedState.angle.getRotations();
        double steerOutput = steerPID.calculate(currentAngleRotations, desiredAngleRotations);
        steerOutput = MathUtil.clamp(steerOutput, -0.5, 0.5);
        steerMotor.set(steerOutput);
        // Druve using optimized speed
        setDriveSpeedMetersPerSecond(optimizedState.speedMetersPerSecond);
    }

    // read steering angle
    public double getSteeringAngleRotations() {
        var encoderPosition = absoluteEncoder.getAbsolutePosition().refresh();

        if (!encoderPosition.getStatus().isOK()) {
            return 0.0;
        }

        double rawPosition = encoderPosition.getValueAsDouble();
        double correctedPosition = rawPosition - encoderOffset;

        return ((correctedPosition + 0.5) % 1.0 + 1.0) % 1.0 - 0.5;
    }

    private void configureCANcoder() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        absoluteEncoder.getConfigurator().apply(encoderConfig);

        absoluteEncoder.getAbsolutePosition().setUpdateFrequency(50);
    }

    public double getSteeringAngleRadians() {
        return getSteeringAngleRotations() * 2.0 * Math.PI;
    }

    public double getSteeringAngleDegrees() {
        return getSteeringAngleRotations() * 360.0;
    }

    public void updateDashboard(String moduleName) {
        SmartDashboard.putNumber(
                moduleName + " Steering Angle (deg)",
                getSteeringAngleDegrees());
    }

    public void setDriveSpeedMetersPerSecond(double speed) {
        // convert linear speed into wheel rps
        double wheelRotationsPerSecond = speed / (2.0 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS);
        // convert wheel speed into motor rps
        double motorRotationsPerSecond = wheelRotationsPerSecond * SwerveConstants.DRIVE_GEAR_RATIO;
        // Updates the phoenix 6 velocity control request
        driveRequest.withVelocity(motorRotationsPerSecond);
        // sends request to TalonFX
        driveMotor.setControl(driveRequest);
    }
}
