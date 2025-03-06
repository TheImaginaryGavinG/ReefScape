package Subsystems;

import Subsystems.Limelight;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {


    public static final String LIMELIGHT_NAME = "limelight";
    public static final double LIMELIGHT_DISTANCE_TO_TARGET = 3.0; // Distance to target in meters
    private final NetworkTable table;

    public Limelight() {
        // Connect to the Limelight NetworkTable, maybe..
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTx() {
        // Get the horizontal offset from the target (in degrees)
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        // Get the vertical offset from the target (in degrees)
        return table.getEntry("ty").getDouble(0.0);
    }

    public double getTa() {
        // Get the area of the target
        return table.getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget() {
        // Returns true if the Limelight is seeing a target
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    

    ///////*****MIGHT BE VERY IMPORTANT****->->->->-> */
        public boolean isAprilTagInRange(double d) {
            // Check if an AprilTag is in range
            return true;  // Placeholder logic
        }
public boolean isTargetVisible(){
//-logic- - conditions-
    return true;
}
    
public void autoAlignToAprilTag(){
    //**auto aligning method**//
///temporary Arm parameter///
}
////*****auto-aligning method (camera is centered, or more than that?)*****////


}
///look at Crescendo Limelight Folder for reference///