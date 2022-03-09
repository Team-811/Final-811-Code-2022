package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
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

    //Removed Current PID Implementation & Reverted to LimelightAim.java
    double kP = 0.5;
    private double gkP = Constants.LIMELIGHT_PID[0];
    private double gkI = Constants.LIMELIGHT_PID[1];
    private double gkD = Constants.LIMELIGHT_PID[2];
    private double gIntegral;
    private double gprevious_error;
    private double grcw;
    
    // private PIDController OutputPID = new PIDController(lkP, lkI, lkD);

    //private MecanumDrive driveTrain = new MecanumDrive(topLeftMotor, bottomLeftMotor, topRightMotor, bottomRightMotor);
    private AHRS gyro;

    /*A new Instance of the Drivetrain*/
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
        //invertGyro(false);
        
    }


    private double SpeedScale = Constants.DRIVETRAIN_SPEED_SCALE;

    public void driveWithMisery(double leftStick, double rightStick, double rotation){
        // if (RobotContainer.driveController.xButton.get())
            //  rotation -= lrcw *0.2;
        //// if (RobotContainer.driveController.yButton.get())
        ////      rotation -= srcw *0.2;
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
        // if (RobotContainer.driveController.xButton.get())
            //  rotation -= lrcw *0.2;
        //// if (RobotContainer.driveController.yButton.get())
        //      rotation -= srcw *0.13;
        double forwardValue = leftStick * SpeedScale;
        double rotationValue = rotation * SpeedScale * 0.8;
        double leftValue = forwardValue + rotationValue ;
        double rightValue = forwardValue - rotationValue;
         topLeftMotor.set(ControlMode.PercentOutput, leftValue + FL);
         bottomLeftMotor.set(ControlMode.PercentOutput, leftValue + BL);
         topRightMotor.set(ControlMode.PercentOutput, -rightValue + FR);
         bottomRightMotor.set(ControlMode.PercentOutput, -rightValue + BR); 
    }

    public void adjustRotation(double rightstickX) {
        robotRotation += (rightstickX * 5);
    }

    public void driveToJoy(double leftStickX, double leftStickY) {
        double angle = ((Math.atan2(leftStickY, leftStickX))* 57);    
        if (angle <=0)
            angle = Math.abs(angle);
        else
            angle = 360 - angle;
        //subtraction goes here
        if(angle > 360)
            angle -= 360;
        if (angle < 0)
            angle += 360;
        double power = (Math.abs(leftStickX) + Math.abs(leftStickY)) / 2; 
        double xDir;
        double yDir;
        xDir = power * (Math.cos(angle) *(Math.PI / 180));
        yDir = power * (Math.sin(angle) *(Math.PI / 180));
        double dtSpeedScale = 0.1;
        double forwardspeed = yDir * Constants.IDEAL_MECHANUM_FORWARDS * dtSpeedScale;
        double strafeSpeed = xDir * Constants.IDEAL_MECHANUM_LEFT * dtSpeedScale;
        if(angle == 0){
            
        }else if(angle == 90){

        }else if(angle == 180){

        }else if(angle == 270){

        }else if(getQuad(angle) == 1){

        }else if(getQuad(angle) == 2){

        }else if(getQuad(angle) == 3){

        }else if(getQuad(angle) == 4){

        }
        double backLeftSpeed = forwardspeed - strafeSpeed;
        double backRightSpeed = forwardspeed - strafeSpeed;
        double frontLeftSpeed = forwardspeed + strafeSpeed;
        double frontRightSpeed = forwardspeed + strafeSpeed;
        topLeftMotor.set(frontLeftSpeed);
        topRightMotor.set(frontRightSpeed);
        bottomRightMotor.set(backRightSpeed);
        bottomLeftMotor.set(backLeftSpeed);
        SmartDashboard.putNumber("Angle", angle);
        SmartDashboard.putNumber("joyx", leftStickX);
        SmartDashboard.putNumber("joyy", leftStickY);
    }

    public double getGyroAngle() {
        double gyroangle = gyro.getAngle();
        double gyroscale = gyroangle % 360;
        gyroangle /= gyroscale;
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

    public void leftWheelsForward(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }
    
    public void rightWheelsForward(double speed){
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void leftWheelsBackwards(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
    }
    
    public void rightWheelsBackwards(double speed){
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void mechanumWHeelRight(double speed){
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
    }

    @Override
    public void periodic() {
        PIDGyro();
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

    public void PIDGyro() {
        grcw = 0;
        double error = robotRotation - getGyroAngle();
        this.gIntegral += (error*0.02);
        double derivative = (error-this.gprevious_error)/0.02;
        grcw = gkP* error + gkI * this.gIntegral + gkD * derivative;
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
