package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import frc.robot.Robot;
// import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.VisionProcessing.Distance;
import frc.robot.VisionProcessing.Limelight;
import frc.robot.VisionProcessing.Lemonlight;

public class Drivetrain extends SubsystemBase implements ISubsystem {

    private WPI_TalonFX topLeftMotor;
    private WPI_TalonFX topRightMotor;
    private WPI_TalonFX bottomLeftMotor;
    private WPI_TalonFX bottomRightMotor;

    public MotorControllerGroup leftMotors = new MotorControllerGroup(topLeftMotor, bottomLeftMotor);
    public MotorControllerGroup rightMotors = new MotorControllerGroup(topRightMotor, bottomRightMotor);
    
    // private double robotRotation = 0;

    // double kP = 0.5;
    // private double gkP = Constants.GYRO_PID[0];
    // private double gkI = Constants.GYRO_PID[1];
    // private double gkD = Constants.GYRO_PID[2];
    // private double gIntegral;
    // private double gprevious_error;
    // private double grcw;

    private double driverControllerAngle = 0;    
    
    private AHRS gyro;

    public Drivetrain(){
        resetSubsystem();
        topLeftMotor = new WPI_TalonFX(RobotMap.DRIVE_TRAIN_TOP_LEFT );
        topRightMotor= new WPI_TalonFX(RobotMap.DRIVE_TRAIN_TOP_RIGHT );
        bottomLeftMotor = new WPI_TalonFX(RobotMap.DRIVE_TRAIN_BOTTOM_LEFT );
        bottomRightMotor= new WPI_TalonFX(RobotMap.DRIVE_TRAIN_BOTTOM_RIGHT );
        topLeftMotor.set(ControlMode.PercentOutput, 0.0f);
        topRightMotor.set(ControlMode.PercentOutput, 0.0f);
        bottomLeftMotor.set(ControlMode.PercentOutput, 0.0f);
        bottomRightMotor.set(ControlMode.PercentOutput, 0.0f);

        topLeftMotor.setInverted(false);
        topRightMotor.setInverted(true);
        bottomLeftMotor.setInverted(false);
        bottomRightMotor.setInverted(true);

        gyro = new AHRS();
        gyro.reset();
        gyro.zeroYaw();        
    }

    public void driveControllerAngle() {
        driverControllerAngle = ((Math.atan2(RobotContainer.driveController.leftStick.getY(), RobotContainer.driveController.leftStick.getX()))* 57);    
        if (driverControllerAngle <=0)
            driverControllerAngle = Math.abs(driverControllerAngle);
        else
            driverControllerAngle = 360 - driverControllerAngle;

        // driverControllerAngle = (Math.atan2(-RobotContainer.driveController.leftStick.getY(), RobotContainer.driveController.leftStick.getX()));
        // if (RobotContainer.driveController.leftStick.getX() < 0.03 && -RobotContainer.driveController.leftStick.getY() < 0.03 && RobotContainer.driveController.leftStick.getX() > -0.03 && -RobotContainer.driveController.leftStick.getY() > -0.03) {
        //     driverControllerAngle = 0;
        

        SmartDashboard.putNumber("X", RobotContainer.driveController.leftStick.getX());
        SmartDashboard.putNumber("Y", -RobotContainer.driveController.leftStick.getY());
    }

    public void driveToJoy(double leftStickY, double leftStickX, double rotation) {
        
        double power = Math.hypot(Math.abs(leftStickX), Math.abs(leftStickY)); 

        double angle = driverControllerAngle - gyro.getAngle();
        
        double ADPower = power * Math.sqrt(2) * 0.5 * (Math.sin(angle) + Math.cos(angle));
        double BCPower = power * Math.sqrt(2) * 0.5 * (Math.sin(angle) - Math.cos(angle));

        // check if turning power will interfere with normal translation
        // check ADPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
        double turningScale = Math.max(Math.abs(ADPower + rotation), Math.abs(ADPower - rotation));
        // check BCPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + rotation), Math.abs(BCPower - rotation)));

        // adjust turn power scale correctly
        if (Math.abs(turningScale) < 1.0)
        {
            turningScale = 1.0;
        }

        // set the motors, and divide them by turningScale to make sure none of them go over the top, which would alter the translation angle
        topLeftMotor.set(ControlMode.PercentOutput, (ADPower - turningScale) / turningScale);
        bottomLeftMotor.set(ControlMode.PercentOutput, (BCPower - turningScale) / turningScale);
        topRightMotor.set(ControlMode.PercentOutput, (BCPower + turningScale) / turningScale);
        bottomRightMotor.set(ControlMode.PercentOutput, (ADPower + turningScale) / turningScale);
    }

    public double getGyroAngle() {
        double gyroangle = gyro.getAngle();
        double gyroscale = gyroangle % 360;
        gyroangle /= gyroscale;
        // Add this to make gyro degree match controller degree
        if (gyroangle >= 270)
	        gyroangle = Math.abs(270 - gyroangle);
        else 
	        gyroangle += 90;
        return gyroangle;
    }

    // public void PIDGyro() {
    //     grcw = 0;
    //     double error = robotRotation - getGyroAngle();
    //     this.gIntegral += (error*0.02);
    //     double derivative = (error-this.gprevious_error)/0.02;
    //     grcw = gkP* error + gkI * this.gIntegral + gkD * derivative;
    // }

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
    public void periodic() {
        // PIDGyro();
        driveControllerAngle();
    }

    @Override
    public void outputSmartdashboard() {
        // driveToJoy(RobotContainer.driveController.getX(), RobotContainer.driveController.getY());
        // SmartDashboard.putNumber("Front Right Wheel", -topRightMotor.getMotorOutputPercent());
        // SmartDashboard.putNumber("Back Left Wheel", bottomLeftMotor.getMotorOutputPercent());
        // SmartDashboard.putNumber("Back Right Wheel", -bottomRightMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("Snakeye X", Lemonlight.getX());
        SmartDashboard.putNumber("Shooter Speed", Distance.ShooterSpeed());
        SmartDashboard.putNumber("XVel", gyro.getVelocityX());
        SmartDashboard.putNumber("YVel", gyro.getVelocityY());
        SmartDashboard.putNumber("ZVel", gyro.getVelocityZ());
        
        if (Limelight.getV() == 1.0)
            SmartDashboard.putBoolean("HasTarget", true);
        else
            SmartDashboard.putBoolean("HasTarget", false);
    }

    @Override
    public void zeroSensors() {
        gyro.zeroYaw();
    }

    @Override
    public void resetSubsystem() {}

    @Override
    public void testSubsystem() {}

}
