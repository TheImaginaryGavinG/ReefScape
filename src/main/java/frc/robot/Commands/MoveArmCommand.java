package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MoveArmCommand extends CommandBase {
    private final Arm arm;
    private final XboxController controller;

    public MoveArmCommand(Arm arm, XboxController controller) {
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }


    public void execute() {
        double manualInput = controller.getLeftY(); // Example: Y-axis for manual control
        arm.moveArm(manualInput); // Allow manual motor movement

        // Adjust height to presets using bumpers
        if (controller.getRawButtonPressed(XboxController.Button.kBumperRight.value)) {
            arm.raiseArm();
        }
        if (controller.getRawButtonPressed(XboxController.Button.kBumperLeft.value)) {
            arm.lowerArm();
        }
    }


    public void end(boolean interrupted) {
        arm.moveArm(0); // Stop the motor when the command ends
        
    }

    
    public boolean isFinished() {
        return false; // Command continuously runs during teleop
    }
}
