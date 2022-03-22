package frc.robot;

public final class Constants { 
    //Drivetrain 
    public static final double DRIVETRAIN_SPEED_SCALE = 0.8;
    public static final double DRIVE_SPEED = 0.1;
    public static final double AIM_SPEED = 0.4;
    public static final double STRAFE_SPEED = 0.8;
    public static final double[] GYRO_PID = {0.004, 0.02, 0.004};
    public static final double[] DRIVE_PID = {0, 0, 0};

    //Swervish Drive
    public static final double IDEAL_MECHANUM_FORWARDS = 0.7;
    public static final double IDEAL_MECHANUM_BACKWARDS = 0.7;
    public static final double IDEAL_MECHANUM_LEFT = 1;
    public static final double IDEAL_MECHANUM_RIGHT = 1;
    
    //limelight
    public static final double OBJECT_AIM_SPEED = 0.4;   
    public static final double CAT_DRIVE_SPEED = 0.15;
    public static final double[] LIMELIGHT_PID_X = {0.004, 0.02, 0.004};
    public static final double[] LIMELIGHT_PID_Y = {0.002, 0.02, 0.004};
    public static final double AUTO_SPEED = 0.2;

    //Intake
    public static final double STORAGE_SPEED = 0.4; 
    public static final double INTAKE_SPEED = 0.45;

    //Shooter
    public static final double SHOOTER_SPEED = 0.9; 

    //Climber
    public static final double ARM_SPEED_SCALE = 0.8;
    public static final double WINCH_SPEED = 0.8;   
}
