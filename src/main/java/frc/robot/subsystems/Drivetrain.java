package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
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
    
    private double robotRotation = 0;

    double kP = 0.5;
    private double gkP = Constants.GYRO_PID[0];
    private double gkI = Constants.GYRO_PID[1];
    private double gkD = Constants.GYRO_PID[2];
    private double gIntegral;
    private double gprevious_error;
    private double grcw;

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
    }

    public void driveToJoy(double leftStickY, double leftStickX, double rotation) {
        //subtraction goes here
        if(driverControllerAngle > 360)
            driverControllerAngle -= 360;
        if (driverControllerAngle < 0)
            driverControllerAngle += 360;
        double power = (Math.abs(leftStickX) + Math.abs(leftStickY)) / 2; 
        double xDir;
        double yDir;
        xDir = power * (Math.cos(driverControllerAngle) *(Math.PI / 180));
        yDir = power * (Math.sin(driverControllerAngle) *(Math.PI / 180));
        double forwardspeed = yDir * Constants.IDEAL_MECHANUM_FORWARDS * Constants.DRIVETRAIN_SPEED_SCALE;
        double strafeSpeed = xDir * Constants.IDEAL_MECHANUM_LEFT * Constants.DRIVETRAIN_SPEED_SCALE;
        if(driverControllerAngle == 0){

        } else if (driverControllerAngle == 90){

        } else if (driverControllerAngle == 180){

        } else if (driverControllerAngle == 270){

        } else if (getQuad(driverControllerAngle) == 1){

        } else if (getQuad(driverControllerAngle) == 2){

        } else if (getQuad(driverControllerAngle) == 3){

        } else if (getQuad(driverControllerAngle) == 4){

        }
        rotation -= grcw;
        double backLeftSpeed = forwardspeed - strafeSpeed;
        double backRightSpeed = forwardspeed - strafeSpeed;
        double frontLeftSpeed = forwardspeed + strafeSpeed;
        double frontRightSpeed = forwardspeed + strafeSpeed;
        topLeftMotor.set(ControlMode.PercentOutput, frontLeftSpeed);
        topRightMotor.set(ControlMode.PercentOutput, frontRightSpeed);
        bottomRightMotor.set(ControlMode.PercentOutput, backRightSpeed);
        bottomLeftMotor.set(ControlMode.PercentOutput, backLeftSpeed);
    }

    public double getGyroAngle() {
        double gyroangle = gyro.getAngle();
        double gyroscale = gyroangle % 360;
        gyroangle /= gyroscale;
        // Add this to make gyro degree match controller degree
        // if (gyroangle >= 270)
	    //     gyroangle = Math.abs(270 - gyroangle);
        // else 
	    //     gyroangle += 90;
        return gyroangle;
    }

    public int getQuad(Double angle) {
        if (angle > 0 && angle < 90)
            return 1;
        else if (angle > 90 && angle < 180)
            return 2;
        else if (angle > 180 && angle < 270)
            return 3;
        else if (angle > 270)
            return 4;
        else
            return 0;
             
    }

    public void PIDGyro() {
        grcw = 0;
        double error = robotRotation - getGyroAngle();
        this.gIntegral += (error*0.02);
        double derivative = (error-this.gprevious_error)/0.02;
        grcw = gkP* error + gkI * this.gIntegral + gkD * derivative;
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
    public void periodic() {
        PIDGyro();
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
