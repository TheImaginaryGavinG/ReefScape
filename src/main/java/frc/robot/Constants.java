package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import Subsystems.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * This class provides a centralized place for all robot-wide constants,
 * such as motor CAN IDs, PID coefficients, geometry, and sensor configurations.
 */
public final class Constants {

    // ---- Drive Constants ----
    public static final class DriveConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI;
        public static final double kDirectionSlewRate = 1.2;
        public static final double kMagnitudeSlewRate = 1.8;
        public static final double kRotationalSlewRate = 2.0;
        public static final double kTrackWidth = Units.inchesToMeters(18.5);
        public static final double kWheelBase = Units.inchesToMeters(32);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

        public static final class ModuleConstants {
            public static final int kDrivingMotorPinionTeeth = 13;
            public static final boolean kTurningEncoderInverted = true;
            public static final double kWheelDiameterMeters = 0.0762;
            public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
            public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
            public static final double kDriveWheelFreeSpeedRps = (NeoMotorConstants.kFreeSpeedRpm / 60) * kWheelCircumferenceMeters / kDrivingMotorReduction;

            public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
            public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;
            public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
            public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;

            public static final double kDrivingP = 0.1;
            public static final double kDrivingI = 0;
            public static final double kDrivingD = 0;
            public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
            public static final double kDrivingMinOutput = -1;
            public static final double kDrivingMaxOutput = 1;

            public static final double kTurningP = 2.6;
            public static final double kTurningI = 0;
            public static final double kTurningD = 0;
            public static final double kTurningFF = 0;
            public static final double kTurningMinOutput = -1;
            public static final double kTurningMaxOutput = 1;

            public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
            public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
            public static final int kDrivingMotorCurrentLimit = 50;
            public static final int kTurningMotorCurrentLimit = 20;
        }

        public static final boolean kGyroReversed = false;

        public static final int kFrontLeftDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 7;
        public static final int kFrontRightDrivingCanId = 39;
        public static final int kRearRightDrivingCanId = 5;

        public static final int kFrontLeftTurningCanId = 4;
        public static final int kRearLeftTurningCanId = 8;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 6;
    }

    public static final class LimelightConstants {
        public static final String kCameraName = "limelight";
        public static final double kTargetHeightMeters = 0.5;
        public static final double kCameraHeightMeters = 0.9;
        public static final double kCameraPitchDegrees = 25.0;
    }


        public static final class ArmConstants {
            public static final int kArmMotorCanId = 9;
            public static final double kArmLengthMeters = 0.6;
            public static final double kArmMaxAngleDegrees = 180;
            public static final double kArmMinAngleDegrees = 0;
            public static final double kArmSpeed = 0.5;
            public static final double kArmP = 0.1;
            public static final double kArmI = 0.0;
            public static final double kArmD = 0.0;
            public static final double kArmFF = 0.2; // Add feedforward constant
            public static final double maxVelocity = 1.0;
            public static final double maxAcceleration = 0.5;
            public static final double maxPosition = 1.0; // You can use a real value for the arm length
            public static final double maxTime = Math.sqrt(3 * maxPosition / maxVelocity);
    }

    public static final class SensorConstants {
        public static final int kFrontDistanceSensorPort = 1;
        public static final int kRearDistanceSensorPort = 2;
        public static final int kColorSensorPort = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}