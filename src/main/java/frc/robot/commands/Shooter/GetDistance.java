package frc.robot.commands.Shooter;

import frc.robot.NetworkTables.LimelightFetch;

public class GetDistance {

    public static double Distance()
    {
        double angle;
        if (LimelightFetch.getY() == 0)
            angle = 0;        
        else
            angle = 30;
        return (6 / Math.tan((angle + Math.abs(LimelightFetch.getY()-14)) *(Math.PI / 180)));
    }

    public static float ShooterSpeed(){
        return (float) (Distance() * 0.1);
    }
}