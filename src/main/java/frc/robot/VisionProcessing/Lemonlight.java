package frc.robot.VisionProcessing;

import org.photonvision.PhotonCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class Lemonlight extends TimedRobot{
    static PhotonCamera camera = new PhotonCamera("Team811ObjectCamera");

    static private NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Team811ObjectCamera");
    static private NetworkTableEntry tx = table.getEntry("targetYaw");
    static private NetworkTableEntry ty = table.getEntry("targetPitch");
    static private NetworkTableEntry ta = table.getEntry("targetArea");

    static private double x = tx.getDouble(0.0);
    static private double y = ty.getDouble(0.0);
    static private double area = ta.getDouble(0.0);


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

    public static boolean getV() {
        tx = table.getEntry("targetYaw");
        x = tx.getDouble(0.0);
        ty = table.getEntry("targetPitch");
        y = ty.getDouble(0.0);
        if (x != 0 || y != 0)
            return true;
        else 
            return false;    
    }
    public static void setTeam() {
        if(TeamSelector.getTeam()){
            camera.setPipelineIndex(1);
            System.out.println("Setting team to Red");
        }else{
            camera.setPipelineIndex(0);
            System.out.println("Setting team to Blue");
        }
    }
}
