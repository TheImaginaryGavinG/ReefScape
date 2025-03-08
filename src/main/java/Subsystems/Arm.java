package Subsystems;
 /* 
S-CURVE:::::::



import com.revrobotics.spark.SparkMax; 
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;

public class Arm extends SubsystemBase {
    private final Joystick controller = new Joystick(0);
    private final SparkMax slideMotor = new SparkMax(9, MotorType.kBrushless);
    private final Encoder slideEncoder = new Encoder(0, 1);

    private double currentTime = 0.0;
    private double startTime = 0.0;

    // Arrays for X, Y, Z coordinates
    private final double[] targetXs = {0.0, 0.5, 1.0, 1.5};
    private final double[] targetYs = {0.0, 0.0, 0.0, 0.0};
    private final double[] targetZs = {0.0, 0.1, 0.2, 0.3};

    private int targetHeight = 0;
    private Translation3d currentTarget = new Translation3d(targetXs[0], targetYs[0], targetZs[0]);

    private final ProfiledPIDController pidController = new ProfiledPIDController(
        1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 1.0)
    );

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.2);

    public Arm() {
        slideEncoder.reset();
    }

    public double convertVoltageToRadians(double voltage) {
        double freeSpeedRPM = 5676;
        double gearRatio = 6.75;
        double freeSpeedRadPerSec = (freeSpeedRPM / 60.0) * (2 * Math.PI);
        return (voltage / 12.0) * freeSpeedRadPerSec / gearRatio;
    }

    private void computeMotionControl() {
        if (startTime == 0.0) {
            startTime = Timer.getFPGATimestamp(); ///?potentially more automated/accurate than knowing position
        }

        currentTime = Timer.getFPGATimestamp() - startTime;

        if (currentTime >= 0.00001 && currentTime <= 2.0) {
            double ratio = currentTime / 2.0;
            double position = 1.0 * Math.pow(ratio, 3) * (10 - 15 * ratio + 6 * ratio * ratio);
            double velocity = 2.0 * Math.pow(ratio, 2) * (30 - 60 * ratio + 30 * ratio * ratio);
            
            double currentPosition = slideEncoder.getDistance();
            double pidOutput = pidController.calculate(currentPosition, position);
            double velocityInRadians = convertVoltageToRadians(velocity);
            double motorOutput = pidOutput + feedforward.calculate(velocityInRadians);

            slideMotor.setVoltage(motorOutput);

            SmartDashboard.putNumber("Position", position);
            SmartDashboard.putNumber("Velocity", velocity);
            SmartDashboard.putNumber("Motor Output", motorOutput);
        } else {
            resetTime();
            slideMotor.setVoltage(0);
        }
    }

    public void trackButtonPresses() {
        if (controller.getRawButtonPressed(1)) {
            targetHeight = Math.min(targetHeight + 1, targetXs.length - 1);
            updateCurrentTarget();
        }
        if (controller.getRawButtonPressed(2)) {
            targetHeight = Math.max(targetHeight - 1, 0);
            updateCurrentTarget();
        }
    }

    private void updateCurrentTarget() {
        currentTarget = new Translation3d(targetXs[targetHeight], targetYs[targetHeight], targetZs[targetHeight]);
    }

    public void resetTime() {
        currentTime = 0.00001;
        startTime = Timer.getFPGATimestamp();
    }

    public void manualControl() {
        double manualInput = controller.getRawAxis(1);
        if (Math.abs(manualInput) > 0.1) {
            slideMotor.set(manualInput);
        } else {
            slideMotor.set(0);
        }
    }

    public void moveToXYZTargetLocation(Translation3d targetPosition) {
        double targetPositionZ = targetPosition.getZ();
        double currentPosition = slideEncoder.getDistance();
        double pidOutput = pidController.calculate(currentPosition, targetPositionZ);
        
        double targetVelocity = pidController.getSetpoint().velocity;
        double motorOutput = pidOutput + feedforward.calculate(targetVelocity);

        slideMotor.setVoltage(motorOutput);

        SmartDashboard.putNumber("Target Position Z", targetPositionZ);
        SmartDashboard.putNumber("Current Arm Position", slideEncoder.getDistance());
        SmartDashboard.putNumber("Motor Output", motorOutput);
    }
    public double[] getTargetPosition() {
        return new double[]{currentTarget.getX(), currentTarget.getY(), currentTarget.getZ()};
    }
    public double getCurrentPosition() {
        return slideEncoder.getDistance();
    }
    
    @Override
    public void periodic() {
        trackButtonPresses();
        computeMotionControl();
        manualControl();

        SmartDashboard.putNumber("Target Position X", currentTarget.getX());
        SmartDashboard.putNumber("Target Position Y", currentTarget.getY());
        SmartDashboard.putNumber("Target Position Z", currentTarget.getZ());
        SmartDashboard.putNumber("Current Arm Position", slideEncoder.getDistance());
        SmartDashboard.putNumber("Arm Motor Voltage", slideMotor.get() * slideMotor.getAppliedOutput());
        SmartDashboard.putNumber("Arm Target Height", targetHeight);
        SmartDashboard.putNumber("Current Time", currentTime);
    }
} 

TRAPEZOIDAL VERSION:::::





 */

import com.revrobotics.spark.SparkMax; 
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final CANSparkMax slideMotor = new CANSparkMax(1, MotorType.kBrushless);
    private final DigitalInput upperLimitSwitch = new DigitalInput(2);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(3);
    
    private double currentSpeed = 0.0;
    private static final double MAX_SPEED = 1.0;
    private static final double ACCELERATION_RATE = 0.02; // Adjust this for smoother acceleration

    public Arm() {
        slideEncoder.reset();
    }

    public void moveArm(double desiredSpeed) {
        // Ensure speed is within allowed range
        if (desiredSpeed > MAX_SPEED) {
            desiredSpeed = MAX_SPEED;
        } else if (desiredSpeed < -MAX_SPEED) {
            desiredSpeed = -MAX_SPEED;
        }

        // Apply acceleration/deceleration
        if (currentSpeed < desiredSpeed) {
            currentSpeed += ACCELERATION_RATE;
            if (currentSpeed > desiredSpeed) {
                currentSpeed = desiredSpeed;
            }
        } else if (currentSpeed > desiredSpeed) {
            currentSpeed -= ACCELERATION_RATE;
            if (currentSpeed < desiredSpeed) {
                currentSpeed = desiredSpeed;
            }
        }

        // Ensure motor stops if limit switches are reached
        if ((currentSpeed > 0 && isAtUpperLimit()) || (currentSpeed < 0 && isAtLowerLimit())) {
            currentSpeed = 0.0;
        }

        slideMotor.set(currentSpeed);
    }

    public void stop() {
        slideMotor.set(0);
    }

    private boolean isAtUpperLimit() {
        return !upperLimitSwitch.get(); // Returns false when pressed
    }

    private boolean isAtLowerLimit() {
        return !lowerLimitSwitch.get(); // Returns false when pressed
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("At Upper Limit", isAtUpperLimit());
        SmartDashboard.putBoolean("At Lower Limit", isAtLowerLimit());
        SmartDashboard.putNumber("Current Arm Speed", currentSpeed);
    }