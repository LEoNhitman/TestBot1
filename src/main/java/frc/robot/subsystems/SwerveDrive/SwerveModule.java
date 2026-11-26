
package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {

    // Communicates with either real hardware or simulated hardware.
    private final SwerveModuleIO io;

    // Stores the latest readings received from the IO layer.
    private final SwerveModuleIO.SwerveModuleIOInputs inputs = new SwerveModuleIO.SwerveModuleIOInputs();

    // PID controller responsible for rotating the module to its target angle.
    private final PIDController steerPID = new PIDController(0.01, 0.0, 0.0);

    // identifies the swerve module
    private int id;

    // Constructor receives the IO implementation instead of creating hardware.
    public SwerveModule(SwerveModuleIO io, int id) {
        this.io = io;
        this.id = id;

        // Allows the PID controller to recognize that angles near
        // -0.5 and +0.5 rotations are close together.
        steerPID.enableContinuousInput(-0.5, 0.5);
    }

    // Reads the latest information from the hardware or simulation.
    public void updateInputs() {
        io.updateInputs(inputs);
    }

    // Calculates and applies the desired steering angle and drive speed.
    public void setDesiredState(SwerveModuleState state) {

        // Read the latest encoder and motor information.
        updateInputs();

        // Get the current steering angle in radians.
        double currentAngle = getSteeringAngleRadians();

        // Optimize the state to minimize how far the module needs to rotate.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
                state,
                new Rotation2d(currentAngle));

        // Get the current and desired steering angles in rotations.
        double currentAngleRotations = getSteeringAngleRotations();
        double desiredAngleRotations = optimizedState.angle.getRotations();

        // Calculate the steering motor output using PID.
        double steerOutput = steerPID.calculate(
                currentAngleRotations,
                desiredAngleRotations);

        // Limit steering output to prevent excessively large commands.
        steerOutput = MathUtil.clamp(steerOutput, -0.5, 0.5);

        // Send the steering command through the IO layer.
        io.setSteerOutput(steerOutput);

        // Send the optimized drive speed through the IO layer.
        setDriveSpeedMetersPerSecond(
                optimizedState.speedMetersPerSecond);
    }

    // Returns the current steering angle in rotations.
    public double getSteeringAngleRotations() {
        return inputs.steerPositionRotations;
    }

    // Converts the steering angle from rotations to radians.
    public double getSteeringAngleRadians() {
        return getSteeringAngleRotations() * 2.0 * Math.PI;
    }

    // Converts the steering angle from rotations to degrees.
    public double getSteeringAngleDegrees() {
        return getSteeringAngleRotations() * 360.0;
    }

    // Displays the module's steering angle on SmartDashboard.
    public void updateDashboard(String moduleName) {
        SmartDashboard.putNumber(
                moduleName + " Steering Angle (deg)",
                getSteeringAngleDegrees());
    }

    // Sends a desired wheel velocity to the IO layer.
    // The real hardware implementation handles gear ratio and wheel conversions.
    public void setDriveSpeedMetersPerSecond(double speed) {
        io.setDriveVelocity(speed);
    }

    // Stops the drive and steering motors.
    public void stop() {
        io.stop();
    }
}
