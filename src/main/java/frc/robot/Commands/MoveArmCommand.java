package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.Arm;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class MoveArmCommand extends Command {
    private final Subsystems.Arm arm;
    private final XboxController controller;

    public MoveArmCommand(Arm arm, XboxController controller) {
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }


    public void execute() {
        // Manual control using joystick (Y-axis)
        double manualInput = controller.getLeftY(); // Example: Y-axis for manual control
        arm.moveArm(manualInput); // Send input to the arm motor for manual movement

        // Proportional raising and lowering using bumpers (specifically for the arm motor)
        if (controller.getRawButton(6)) { // Right Bumper is held
            arm.moveArm(0.5); // Raise the arm motor at half speed
        } else if (controller.getRawButton(5)) { // Left Bumper is held
            arm.moveArm(-0.5); // Lower the arm motor at half speed
        } else {
            arm.moveArm(0); // Stop the arm motor when no button is pressed
        }
    }

    public void end(boolean interrupted) {
        arm.stop();; // Stop the motor when the command ends
        
        
    }

    
    public boolean isFinished() {
        return false; // Command continuously runs during teleop
    }
}
