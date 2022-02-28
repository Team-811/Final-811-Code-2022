package frc.robot.VisionProcessing;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class TeamSelector extends TimedRobot{
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("FMSInfo");
    static NetworkTableEntry alliance = table.getEntry("IsRedAlliance");

    static boolean isRed = alliance.getBoolean(false);

    public static boolean getTeam() {
        alliance = table.getEntry("IsRedAlliance");
        isRed = alliance.getBoolean(false);
        return isRed;
    }

    
}
