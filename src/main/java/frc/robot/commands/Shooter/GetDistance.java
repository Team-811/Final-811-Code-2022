package frc.robot.commands.Shooter;

import frc.robot.Vision.LimelightFetch;

public class GetDistance {

    public static double Distance()
    {
        return (6 / (Math.tan(60.0 + LimelightFetch.getY())));
    }
}