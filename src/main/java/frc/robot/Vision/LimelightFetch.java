package frc.robot.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class LimelightFetch extends TimedRobot{
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    static NetworkTableEntry ta = table.getEntry("ta");
    static NetworkTableEntry tv = table.getEntry("tv");

    static double x = tx.getDouble(0.0);
    static double y = ty.getDouble(0.0);
    static double area = ta.getDouble(0.0);
    static double inVision = tv.getDouble(0.0);

    public static double getX() {
        tx = table.getEntry("tx");
        x = tx.getDouble(0.0);
        return x;
    }

    public static double getY() {
        ty = table.getEntry("ty");
        y = ty.getDouble(0.0);
        return y;
    }

    public static double getA() {
        ta = table.getEntry("ta");
        area = ta.getDouble(0.0);
        return area;
    }

    public static double getV() {
        tv = table.getEntry("tv");
        inVision = tv.getDouble(0.0);
        return inVision;
    }

    
}
