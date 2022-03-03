package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants { 
    //Drivetrain 
    public static final double DRIVETRAIN_SPEED_SCALE = 0.6;
    public static final double DRIVE_SPEED = 0.1;
    public static final double AIM_SPEED = 0.4;
    public static final double STRAFE_SPEED = 0.4; 
    public static final double OBJECT_AIM_SPEED = 0.4;   
    public static final double CAT_DRIVE_SPEED = 0.2;
    public static final double[] LIMELIGHT_PID = {0.004, 0.02, 0.004};
    public static final double[] SNAKEEYE_PID = {0.004, 0.02, 0.004};
    public static final double AUTO_SPEED = 0.2;

    //Intake
    public static final double STORAGE_SPEED = 0.3; 
    public static final double INTAKE_SPEED = 0.35;

    //Shooter
    public static final double SHOOTER_SPEED = 0.87; 

    //Climber
    public static final double ARM_SPEED_SCALE = 0.8;
    public static final double WINCH_SPEED = 0.8;   
}
