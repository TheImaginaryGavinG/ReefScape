
package frc.robot;

import Subsystems.Arm;
import Subsystems.Drivetrain;
import Subsystems.Limelight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.geometry.Translation3d;

public class Robot extends TimedRobot {
    private final XboxController m_controller = new XboxController(0);
    private final Drivetrain m_swerve = new Drivetrain();
    private final Arm m_arm = new Arm(); // Declaring Arm as a field
    private final Limelight limelight = new Limelight(); // Move Limelight instantiation here
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    
    private static final double APRILTAGLOCKDISTANCE = 3.0; // Modify for auto-targeting distance
    private boolean autoTargetingEnabled = false;
    private boolean semiAutonomousModeEnabled = false; // New flag for semi-autonomous mode

    @Override
    public void robotInit() {
        // Button to toggle auto-targeting (mapped to A button)
        new JoystickButton(m_controller, XboxController.Button.kA.value)
            .onTrue(new InstantCommand(() -> autoTargetingEnabled = !autoTargetingEnabled));
        
        // Button to toggle semi-autonomous mode (mapped to B button)->->->
      /*  new JoystickButton(m_controller, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() -> semiAutonomousModeEnabled = !semiAutonomousModeEnabled));
    }*/


    @Override
    public void autonomousPeriodic() {
        driveWithJoystick(false);
        m_swerve.updateOdometry();
    }

    @Override
    public void teleopPeriodic() {
        // If semi-autonomous mode is enabled, go to a preset location
        if (semiAutonomousModeEnabled) {
            moveArmToTargetLocation(); // Function that handles moving the arm to a preset position
        } else {
            // Otherwise, manual control is active
            driveWithJoystick(true);
        }

        //m_arm.manualControl(); ??//
        m_swerve.updateOdometry();
        commandScheduler.run();

        // Auto-lock onto target when within X feet
        if (autoTargetingEnabled && limelight.isTargetVisible() && limelight.isAprilTagInRange(APRILTAGLOCKDISTANCE)) {
            m_arm.periodic(); // Call arm periodic method
        }

        // Auto-align to AprilTag if autoTargetingEnabled
        if (autoTargetingEnabled) {
            limelight.autoAlignToAprilTag();
        }
    }

    private void driveWithJoystick(boolean fieldRelative) {
        double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02)) * Drivetrain.kMaxSpeed;
        double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02)) * Drivetrain.kMaxSpeed;
        double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02)) * Drivetrain.kMaxAngularSpeed;

        m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
    }

    private void moveArmToTargetLocation() {
        // Implement logic for the arm to move to a specific target location in semi-autonomous mode
        // This could be done by calculating the desired position based on robot coordinates or predefined waypoints.
       
        // targetHeight is updated based on button presses or other logic
        Translation3d targetPosition = new Translation3d(m_arm.getTargetPosition()[0], m_arm.getTargetPosition()[1], m_arm.getTargetPosition()[2]); // Using indexed variables from Arm
        m_arm.moveToXYZTargetLocation(targetPosition);
    }
}
