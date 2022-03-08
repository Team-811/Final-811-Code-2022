package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
// import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.VisionProcessing.Distance;
import frc.robot.VisionProcessing.Limelight;
import frc.robot.VisionProcessing.Lemonlight;

public class Drivetrain extends SubsystemBase implements ISubsystem {

    private WPI_TalonSRX topLeftMotor;
    private WPI_TalonSRX topRightMotor;
    private WPI_TalonSRX bottomLeftMotor;
    private WPI_TalonSRX bottomRightMotor;


    public MotorControllerGroup leftMotors = new MotorControllerGroup(topLeftMotor, bottomLeftMotor);
    public MotorControllerGroup rightMotors = new MotorControllerGroup(topRightMotor, bottomRightMotor);
    
    //Removed Current PID Implementation & Reverted to LimelightAim.java
    // double kP = 0.5;
    // private double lkP = Constants.LIMELIGHT_PID[0];
    // private double lkI = Constants.LIMELIGHT_PID[1];
    // private double lkD = Constants.LIMELIGHT_PID[2];
    // private double lSetpoint = 0;
    // private double lIntegral;
    // private double lprevious_error;
    // private double lrcw;

    // private double skP = Constants.SNAKEEYE_PID[0];
    // private double skI = Constants.SNAKEEYE_PID[1];
    // private double skD = Constants.SNAKEEYE_PID[2];
    // private double sSetpoint = 0;
    // private double sIntegral;
    // private double sprevious_error;
    // private double srcw;
    
    // private PIDController OutputPID = new PIDController(lkP, lkI, lkD);

    //private MecanumDrive driveTrain = new MecanumDrive(topLeftMotor, bottomLeftMotor, topRightMotor, bottomRightMotor);
    private AHRS gyro;

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

        gyro = new AHRS();
        gyro.reset();
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

         @SuppressWarnings("unused")
         double correction; 
         if (Math.abs(rotation) < 0.2) {
             correction = gyroCorrection();
         } else {
             correction = 0;
         }

         prevAngle = getGyroAngle(); 
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

    public void driveToJoy(double leftStickX, double leftStickY) {
        double angle = ((Math.atan2(leftStickY, leftStickX))* 57);
        if (angle <=0)
            angle = Math.abs(angle);
        else
            angle = 360 - angle;
        double power = (Math.abs(leftStickX) + Math.abs(leftStickY)) / 2; 
        double xDir;
        double yDir;
        xDir = power * (Math.cos(angle));
        yDir = power * (Math.sin(angle));
        double dtSpeedScale = 0.0;
        double forwardspeed = yDir * Constants.IDEAL_MECHANUM_FORWARDS * dtSpeedScale;
        double strafeSpeed = xDir * Constants.IDEAL_MECHANUM_LEFT * dtSpeedScale;
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
        // PIDL();
        // PIDS();    
        // PIDA();
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

    //gyro
    private int gyroInversion = 1;
    private double correctRate = 0.5;
    private double prevAngle = 0;

    public double getGyroAngle() {
        return gyroInversion * gyro.getAngle();
    }

    public double getAngularVelocity() {
        return gyroInversion * gyro.getRate();
    }

    public void gyroInvert(boolean inverted) {
        if (inverted) {
            gyroInversion = -1;
        } else {
            gyroInversion = 1;
        }
    }

    public double gyroCorrection() {
        return (getGyroAngle() - prevAngle) * correctRate; 
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    

    @Override
    public void outputSmartdashboard() {
        driveToJoy(RobotContainer.driveController.getX(), RobotContainer.driveController.getY());
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
        zeroGyro();

    }

    @Override
    public void resetSubsystem() {
        
    }

    @Override
    public void testSubsystem() {

    }

    // public void PIDL() {
    //     double error = lSetpoint - Limelight.getX();
    //     this.lIntegral += (error*0.02);
    //     double derivative = (error-this.lprevious_error)/0.02;
    //     lrcw = lkP* error + lkI * this.lIntegral + lkD * derivative;
        
    // }

    // public void PIDS() {
    //     double error = sSetpoint - Lemonlight.getX();
    //     this.sIntegral += (error*0.02);
    //     double derivative = (error-this.sprevious_error)/0.02;
    //     srcw = skP* error + skI * this.sIntegral + skD * derivative;
        
    // }

    // public void PIDA() {
    //     double error = aSetpoint - gyro.getAngle();
    //     this.aIntegral += (error*0.02);
    //     double derivative = (error-this.aprevious_error)/0.02;
    //     arcw = akP* error + akI * this.aIntegral + akD * derivative;
    // }

    // public void driveAuto(){
    //     double rotation = 3.0 - (arcw *0.2);
    //     double rotationValue = rotation * SpeedScale * 0.8;
    //     double leftValue = rotationValue;
    //     double rightValue = rotationValue;
    //      topLeftMotor.set(ControlMode.PercentOutput, leftValue);
    //      bottomLeftMotor.set(ControlMode.PercentOutput, leftValue);
    //      topRightMotor.set(ControlMode.PercentOutput, -rightValue);
    //      bottomRightMotor.set(ControlMode.PercentOutput, -rightValue);
    // }

    // public void aSetpoint(int point){
    //     aSetpoint = point;
    // }

}
