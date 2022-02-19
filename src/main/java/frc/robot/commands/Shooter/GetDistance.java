package frc.robot.commands.Shooter;

import frc.robot.Vision.LimelightFetch;

public class GetDistance {

    public static double Distance()
    {
        double angle;
        if (LimelightFetch.getY() == 0)
            angle = 0;        
        else
            angle = 30;
        return (6 / Math.tan((angle + Math.abs(LimelightFetch.getY())) *(Math.PI / 180)));
    }
}