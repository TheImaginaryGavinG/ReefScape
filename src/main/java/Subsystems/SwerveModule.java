package Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    // Constants
    private static final double kWheelRadius = 0.0508; // in meters (2 inches)
    private static final int kEncoderResolution = 4096; // for SparkMax

    // Max values for the swerve module (adjust as needed)
    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // rad/s^2 (example)

    // Motors and encoders for the swerve module
    private final SparkMax m_driveMotor;
    private final SparkMax m_turningMotor;

    private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;

    // PID Controllers with configurable constants
    private final PIDController m_drivePIDController;
    private final ProfiledPIDController m_turningPIDController;

    // Feedforward
    private final SimpleMotorFeedforward m_driveFeedforward;
    private final SimpleMotorFeedforward m_turnFeedforward;

    /**
     * Constructs a SwerveModule with configurable PID constants and feedforward.
     *
     * @param driveMotorChannel The CAN ID of the drive motor.
     * @param turningMotorChannel The CAN ID of the turning motor.
     * @param driveEncoderChannelA The DIO channel A of the drive encoder.
     * @param driveEncoderChannelB The DIO channel B of the drive encoder.
     * @param turningEncoderChannelA The DIO channel A of the turning encoder.
     * @param turningEncoderChannelB The DIO channel B of the turning encoder.
     * @param drivePIDConstants The PID constants for the drive motor.
     * @param turningPIDConstants The PID constants for the turning motor.
     * @param driveFeedforwardConstants The feedforward constants for the drive motor.
     * @param turnFeedforwardConstants The feedforward constants for the turning motor.
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel,
                        int driveEncoderChannelA, int driveEncoderChannelB,
                        int turningEncoderChannelA, int turningEncoderChannelB,
                        double[] drivePIDConstants, double[] turningPIDConstants,
                        double[] driveFeedforwardConstants, double[] turnFeedforwardConstants) {

        // Initialize motors
        m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

        // Initialize encoders
        m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
        m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

        // Configure encoders
        m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution); // meters per pulse
        m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution); // radians per pulse

        // Set the PID controller input range for the turning motor
        m_turningPIDController = new ProfiledPIDController(turningPIDConstants[0], turningPIDConstants[1], turningPIDConstants[2], 
            new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Create PID controllers and feedforward with provided constants
        m_drivePIDController = new PIDController(drivePIDConstants[0], drivePIDConstants[1], drivePIDConstants[2]);
        m_driveFeedforward = new SimpleMotorFeedforward(driveFeedforwardConstants[0], driveFeedforwardConstants[1]);
        m_turnFeedforward = new SimpleMotorFeedforward(turnFeedforwardConstants[0], turnFeedforwardConstants[1]);
    }

    public SwerveModule(int i, int j, int k) {
        //TODO Auto-generated constructor stub
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module (speed and angle).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module (distance and angle).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
    }

    /**
     * Sets the desired state for the module, with limits to avoid excessive speeds.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation);

        // Limit speed to avoid excessive velocities
        double maxSpeed = 3.0; // Max speed in meters per second
        double limitedSpeed = Math.min(desiredState.speedMetersPerSecond, maxSpeed);
        desiredState = new SwerveModuleState(limitedSpeed, desiredState.angle);

        // Calculate the drive output from the drive PID controller
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);
        final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller
        final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(), desiredState.angle.getRadians());
        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        // Set the motor voltages
        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput + turnFeedforward);

        // Output current speed, angle, and PID errors to SmartDashboard for debugging
        SmartDashboard.putNumber("Swerve Module Speed", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve Module Angle", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Drive PID Error", m_drivePIDController.getPositionError());
        SmartDashboard.putNumber("Turn PID Error", m_turningPIDController.getPositionError());
    }
}
