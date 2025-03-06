package Subsystems;
/* 
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
} */


import com.revrobotics.spark.SparkMax; 
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final SparkMax slideMotor = new SparkMax(1, MotorType.kBrushless);
    private final Encoder slideEncoder = new Encoder(0, 1);

    private final double DISTANCE_PER_COUNT = 0.01; // cm per encoder count
    private final double MAX_HEIGHT = 100.0;       // Maximum height in cm
    private final double MIN_HEIGHT = 0.0;         // Minimum height in cm

    private final double[] targetZs = {0.0, 50.0, 100.0}; // Preset heights in cm
    private int targetIndex = 0;
    private Translation3d currentTarget = new Translation3d(0, 0, targetZs[targetIndex]);

    public Arm() {
        slideEncoder.reset();
    }

    public void moveArm(double input) {
        double currentHeight = slideEncoder.getDistance() * DISTANCE_PER_COUNT;

        // Limit motor movement within height bounds
        if ((input > 0 && currentHeight < MAX_HEIGHT) || (input < 0 && currentHeight > MIN_HEIGHT)) {
            slideMotor.set(input);
        } else {
            slideMotor.set(0);
        }
    }

    public void raiseArm() {
        targetIndex = Math.min(targetIndex + 1, targetZs.length - 1);
        updateCurrentTarget();
    }

    public void lowerArm() {
        targetIndex = Math.max(targetIndex - 1, 0);
        updateCurrentTarget();
    }

    private void updateCurrentTarget() {
        currentTarget = new Translation3d(0, 0, targetZs[targetIndex]);
    }

    public double getDistanceFromTarget() {
        double currentHeight = slideEncoder.getDistance() * DISTANCE_PER_COUNT;
        return targetZs[targetIndex] - currentHeight;
    }

    public double[] getTargetPosition() {
        return new double[] {0, 0, targetZs[targetIndex]};
    }

    @Override
    public void periodic() {
        // Send feedback to the dashboard
        SmartDashboard.putNumber("Current Arm Height (cm)", slideEncoder.getDistance() * DISTANCE_PER_COUNT);
        SmartDashboard.putNumber("Target Arm Height (cm)", targetZs[targetIndex]);
        SmartDashboard.putNumber("Distance from Target (cm)", getDistanceFromTarget());
    }

    public void moveToXYZTargetLocation(Translation3d target) {
        // Optional: Add logic for fine-grained target movement
    }
}




