package frc.robot.VisionProcessing;

public class Distance {

    public static double get()
    {
        double angle;
        if (Limelight.getY() == 0)
            angle = 0;        
        else
            angle = 42;
            return (6 / Math.tan((angle + Limelight.getY() ) *(Math.PI / 180)));
        }

    public static float ShooterSpeed(){
        return (float) (get() * 0.1);
    }
}