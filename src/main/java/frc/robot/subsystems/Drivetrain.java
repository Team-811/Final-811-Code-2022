package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.NetworkTables.LimelightFetch;
import frc.robot.NetworkTables.SnakeEyesFetch;
// import frc.robot.commands.Shooter.GetDistance;
import frc.robot.commands.Shooter.GetDistance;

public class Drivetrain extends SubsystemBase implements ISubsystem {

    private WPI_TalonSRX topLeftMotor;
    private WPI_TalonSRX topRightMotor;
    private WPI_TalonSRX bottomLeftMotor;
    private WPI_TalonSRX bottomRightMotor;
    // private MotorControllerGroup leftMotors = new MotorControllerGroup(topLeftMotor, bottomLeftMotor);
    // private MotorControllerGroup rigthMotors = new MotorControllerGroup(topRightMotor, topLeftMotor);
    // private MecanumDrive driveTrain = new MecanumDrive(topLeftMotor, bottomLeftMotor, topRightMotor, bottomRightMotor);
    private static AHRS gyro = new AHRS();
    /*A new Instance of the Drivetrain*/
    public Drivetrain(){
    
        resetSubsystem();
        topLeftMotor = new WPI_TalonSRX(RobotMap.DRIVE_TRAIN_TOP_LEFT );
        topRightMotor= new WPI_TalonSRX(RobotMap.DRIVE_TRAIN_TOP_RIGHT );
        bottomLeftMotor = new WPI_TalonSRX(RobotMap.DRIVE_TRAIN_BOTTOM_LEFT );
        bottomRightMotor= new WPI_TalonSRX(RobotMap.DRIVE_TRAIN_BOTTOM_RIGHT );
        topLeftMotor.set(ControlMode.PercentOutput, 0.0f);
        topRightMotor.set(ControlMode.PercentOutput, 0.0f);
        bottomLeftMotor.set(ControlMode.PercentOutput, 0.0f);
        bottomRightMotor.set(ControlMode.PercentOutput, 0.0f);
    }

    private double SpeedScale = Constants.DRIVETRAIN_SPEED_SCALE;
    public void driveWithMisery(double leftStick, double rightStick, double rotation){
        double forwardValue = leftStick * SpeedScale;
        double rotationValue = rotation * SpeedScale * 0.8;
        double leftValue = forwardValue + rotationValue;
        double rightValue = forwardValue - rotationValue;
         topLeftMotor.set(ControlMode.PercentOutput, leftValue);
         bottomLeftMotor.set(ControlMode.PercentOutput, leftValue);
         topRightMotor.set(ControlMode.PercentOutput, -rightValue);
         bottomRightMotor.set(ControlMode.PercentOutput, -rightValue); 
    }

    public void driveWithMisery(double leftStick, double rightStick, double rotation, double FL, double FR, double BL, double BR){
        double forwardValue = leftStick * SpeedScale;
        double rotationValue = rotation * SpeedScale * 0.8;
        double leftValue = forwardValue + rotationValue ;
        double rightValue = forwardValue - rotationValue;
         topLeftMotor.set(ControlMode.PercentOutput, leftValue + FL);
         bottomLeftMotor.set(ControlMode.PercentOutput, leftValue + BL);
         topRightMotor.set(ControlMode.PercentOutput, -rightValue + FR);
         bottomRightMotor.set(ControlMode.PercentOutput, -rightValue + BR); 
    }

    public void leftWheelsForward(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }
    
    public void rightWheelsForward(double speed){
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void mechanumWHeelRight(double speed){
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void mechanumWHeelLeft(double speed){
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void turnLeft(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void turnRight(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopRobot(){
        topLeftMotor.set(ControlMode.PercentOutput, 0);
        bottomLeftMotor.set(ControlMode.PercentOutput, 0);
        topRightMotor.set(ControlMode.PercentOutput, 0);
        bottomRightMotor.set(ControlMode.PercentOutput, 0);
    }

    public void driveForward(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void driveBackwards(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void frontRightForward(double speed){
        topRightMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void backRightForward(double speed){
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void frontLeftForward(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void backLeftForward(double speed){
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void outputSmartdashboard() {
        // SmartDashboard.putNumber("Front Right Wheel", -topRightMotor.getMotorOutputPercent());
        // SmartDashboard.putNumber("Back Left Wheel", bottomLeftMotor.getMotorOutputPercent());
        // SmartDashboard.putNumber("Back Right Wheel", -bottomRightMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("Snakeye X", SnakeEyesFetch.getX());
        SmartDashboard.putNumber("Shooter Speed", GetDistance.ShooterSpeed());
        
        if (LimelightFetch.getV() == 1.0)
            SmartDashboard.putBoolean("HasTarget", true);
        else
            SmartDashboard.putBoolean("HasTarget", false);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void resetSubsystem() {
        
    }

    @Override
    public void testSubsystem() {

    }

    
}
