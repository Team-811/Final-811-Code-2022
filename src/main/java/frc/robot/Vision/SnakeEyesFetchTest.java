package frc.robot.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class SnakeEyesFetchTest extends TimedRobot{
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("Team811ObjectCamera");
    static NetworkTableEntry tx = table.getEntry("targetYaw");
    static NetworkTableEntry ty = table.getEntry("targetPitch");
    static NetworkTableEntry ta = table.getEntry("targetArea");
    static NetworkTableEntry tv = table.getEntry("hasTarget");

    static double x = tx.getDouble(0.0);
    static double y = ty.getDouble(0.0);
    static double area = ta.getDouble(0.0);
    static boolean inVision = tv.getBoolean(false);

    public static double getX() {
        tx = table.getEntry("targetYaw");
        x = tx.getDouble(0.0);
        return x;
    }

    public static double getY() {
        ty = table.getEntry("targetPitch");
        y = ty.getDouble(0.0);
        return y;
    }

    public static double getA() {
        ta = table.getEntry("targetArea");
        area = ta.getDouble(0.0);
        return area;
    }

    public static double getV() {
        tv = table.getEntry("hasTarget");
        if (inVision == true) 
            return 1.0;
        else 
            return 0.0;
    }

    
}
