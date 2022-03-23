package frc.robot.subsystems;


import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import frc.robot.Constants;
// import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.VisionProcessing.Distance;
import frc.robot.VisionProcessing.Limelight;
import frc.robot.VisionProcessing.Lemonlight;

public class Drivetrain extends SubsystemBase implements ISubsystem {
    private AHRS gyro = new AHRS();

    private WPI_TalonFX topLeftMotor;
    private WPI_TalonFX topRightMotor;
    private WPI_TalonFX bottomLeftMotor;
    private WPI_TalonFX bottomRightMotor;

    // private  PIDController LimeXPID = new PIDController(0.04, 0, 0, 0.02);
    

    public MotorControllerGroup leftMotors = new MotorControllerGroup(topLeftMotor, bottomLeftMotor);
    public MotorControllerGroup rightMotors = new MotorControllerGroup(topRightMotor, bottomRightMotor);
    

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));//placeholder amount of inches
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.64267, 5821.3, 579.7); //placeholders
  
    private PIDController leftPIDController = new PIDController(13.661, 0, 0); //Placeholder kp values
    private PIDController rightPIDController = new PIDController(13.661, 0, 0);
  
    private Pose2d pose;
    public static final double kEncoderDistancePerPulse =   2048 / 10.71 * 2 * Math.PI * Units.inchesToMeters(3); // The three is a placeholder wheel radius

    //Removed Current PID Implementation & Reverted to LimelightAim.java
    // double kP = 0.5;
    // private double lxkP = Constants.LIMELIGHT_PID_X[0];
    // private double lxkI = Constants.LIMELIGHT_PID_X[1];
    // private double lxkD = Constants.LIMELIGHT_PID_X[2];
    // private double lxSetpoint = 0;
    // private double lxIntegral;
    // private double lxprevious_error;
    // private double lxrcw;

    // private double lykP = Constants.LIMELIGHT_PID_Y[0];
    // private double lykI = Constants.LIMELIGHT_PID_Y[1];
    // private double lykD = Constants.LIMELIGHT_PID_Y[2];
    // private double lySetpoint = 8.5;
    // private double lyIntegral;
    // private double lyprevious_error;
    // private double lyrcw;

    // private double skP = Constants.SNAKEEYE_PID[0];
    // private double skI = Constants.SNAKEEYE_PID[1];
    // private double skD = Constants.SNAKEEYE_PID[2];
    // private double sSetpoint = 0;
    // private double sIntegral;
    // private double sprevious_error;
    // private double srcw;

    // private double akP = Constants.AUTO_PID[0];
    // private double akI = Constants.AUTO_PID[1];
    // private double akD = Constants.AUTO_PID[2];
    // private double aSetpoint = 180;
    // private double aIntegral;
    // private double aprevious_error;
    // private double arcw;

    private double dkP = Constants.DRIVE_PID[0];
    private double dkI = Constants.DRIVE_PID[1];
    private double dkD = Constants.DRIVE_PID[2];
    private double dSetpoint = 180;
    private double dIntegral;
    private double dprevious_error;
    private double drcw;
    
    // private PIDController OutputPID = new PIDController(lxkP, lkI, lkD);

    //private MecanumDrive driveTrain = new MecanumDrive(topLeftMotor, bottomLeftMotor, topRightMotor, bottomRightMotor);

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
        topLeftMotor.setInverted(false);
        bottomLeftMotor.setInverted(false);
        topRightMotor.setInverted(true);
        bottomRightMotor.setInverted(true);

        gyro.reset();
    }


    private double SpeedScale = Constants.DRIVETRAIN_SPEED_SCALE;

    public void driveWithMisery(double leftStick, double rightStick, double rotation){
        // if (RobotContainer.driveController.xButton.get())
        //      rotation -= lxrcw *0.2;
        // if (RobotContainer.driveController.yButton.get())
        //      leftStick -= lyrcw *0.02;
        // double xPIDSpeed = LimeXPID.calculate(Limelight.getX(), 0);
        
        double forwardValue = leftStick * SpeedScale;
        double rotationValue = rotation * SpeedScale * 0.8;
        double leftValue = forwardValue + rotationValue;
        double rightValue = forwardValue - rotationValue;
         topLeftMotor.set(ControlMode.PercentOutput, leftValue);
         bottomLeftMotor.set(ControlMode.PercentOutput, leftValue);
         topRightMotor.set(ControlMode.PercentOutput, rightValue);
         bottomRightMotor.set(ControlMode.PercentOutput, rightValue);  
    }

    public void driveWithMisery(double leftStick, double rightStick, double rotation, double FL, double FR, double BL, double BR){
        // if (RobotContainer.driveController.xButton.get())
        //      rotation -= lxrcw *0.2;
        // if (RobotContainer.driveController.yButton.get())
        //      leftStick -= lyrcw *0.02;

        //// if (RobotContainer.driveController.yButton.get())
        //      rotation -= srcw *0.13;
        double forwardValue = leftStick * SpeedScale;
        double rotationValue = rotation * SpeedScale * 0.8;
        double leftValue = forwardValue + rotationValue ;
        double rightValue = forwardValue - rotationValue;
         topLeftMotor.set(ControlMode.PercentOutput, leftValue + FL);
         bottomLeftMotor.set(ControlMode.PercentOutput, leftValue + BL);
         topRightMotor.set(ControlMode.PercentOutput, rightValue - FR);
         bottomRightMotor.set(ControlMode.PercentOutput, rightValue - BR); 
    }
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-gyro.getAngle());
      }
    
      public DifferentialDriveWheelSpeeds getSpeeds() {
        System.out.println("Aditya"+ Units.inchesToMeters(topLeftMotor.getSelectedSensorVelocity() * 600 / 2048 / 10.71 * (2 * Math.PI * 3)));

        return new DifferentialDriveWheelSpeeds(
          Units.inchesToMeters(topLeftMotor.getSelectedSensorVelocity() * 600 / 2048 / 10.71 * (2 * Math.PI * 3)), //7.29 is placeholder gear ratio of wheel and 3 is a placeholder wheel radius
          Units.inchesToMeters(topRightMotor.getSelectedSensorVelocity() * 600 / 2048 / 10.71 * 2 * Math.PI * 3)
          
          
        );
      }
    
      public SimpleMotorFeedforward getFeedForward() {
        return feedforward;
      }
    
      public PIDController getleftPidController(){
        return leftPIDController;
      }
    
      public PIDController getrighPidController(){
        return rightPIDController;
      }
    
      public DifferentialDriveKinematics getKinematics() {
        return kinematics;
      }
    
      public Pose2d getPose(){
        return pose;
      }
    
      public Trajectory getDefaultTrajectory(){
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          Arrays.asList(new Pose2d(), new Pose2d(1, 0, new Rotation2d())),
          config
        );
        return trajectory;
      }
    
      public void setOutput(double leftVolts, double rightVolts){
        topLeftMotor.set(ControlMode.PercentOutput, leftVolts/12);
        bottomLeftMotor.set(ControlMode.PercentOutput, leftVolts/12);
        topRightMotor.set(ControlMode.PercentOutput, rightVolts/12);
        bottomRightMotor.set(ControlMode.PercentOutput, rightVolts/12);
      }
    

    
    
      public double getEncoderDistance(WPI_TalonFX motor){
        return Units.inchesToMeters(motor.getSelectedSensorPosition()/2048 / 10.71 * (2 * Math.PI * 3));
      }
    
      
    
      /*
      @param  Pathfile  the file path to your pathweaver.json file example: "paths/YourPath.wpilib.json"  
      */
      public Command generateAutoPath(String Pathfile){
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/YourPath.wpilib.json");
        Trajectory selectedTrajectory;
        try {
            selectedTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          selectedTrajectory = this.getDefaultTrajectory();
          DriverStation.reportError("Unable to open trajectory, Robots should quit, but yours did:", ex.getStackTrace());
        }
        RamseteCommand command = new RamseteCommand(
          selectedTrajectory,
          this::getPose,
          new RamseteController(2, 0.7),
          this.getFeedForward(),
          this.getKinematics(),
          this::getSpeeds,
          this.getleftPidController(),
          this.getrighPidController(),
          this::setOutput,
          this
          );
        return command;
      }
    

    public void leftWheelsForward(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }
    
    public void rightWheelsForward(double speed){
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void mechanumWHeelRight(double speed){
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
    }



    public void mechanumWHeelLeft(double speed){
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void turnLeft(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void turnRight(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
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
        topRightMotor.set(ControlMode.PercentOutput, speed);
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void driveBackwards(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, -speed);
        bottomLeftMotor.set(ControlMode.PercentOutput, -speed);
        topRightMotor.set(ControlMode.PercentOutput, -speed);
        bottomRightMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void driveBackwardsPID(double setpoint){
        dSetpoint = setpoint;
        topLeftMotor.set(ControlMode.PercentOutput, drcw);
        topRightMotor.set(ControlMode.PercentOutput, drcw);
        bottomLeftMotor.set(ControlMode.PercentOutput, drcw);
        bottomRightMotor.set(ControlMode.PercentOutput, drcw);
    }

    public void frontRightForward(double speed){
        topRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void backRightForward(double speed){
        bottomRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void frontLeftForward(double speed){
        topLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void backLeftForward(double speed){
        bottomLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getDistance(){
        return topLeftMotor.getSelectedSensorPosition()  / 2048 / 10.71 * 2 * Math.PI * Units.inchesToMeters(3);
    }
    
    // public void PIDLX() {
    //     double error = lxSetpoint - Limelight.getX();
    //     this.lxIntegral += (error*0.02);
    //     double derivative = (error-this.lxprevious_error)/0.02;
    //     lxrcw = lxkP* error + lxkI * this.lxIntegral + lxkD * derivative;
        
    // }

    // public void PIDLY() {
    //     double error = lySetpoint - Distance.get();
    //     this.lyIntegral += (error*0.02);
    //     double derivative = (error-this.lyprevious_error)/0.02;
    //     lyrcw = lykP* error + lykI * this.lyIntegral + lykD * derivative;
        
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

    public void PIDD() {
        double error = dSetpoint - Math.abs(topLeftMotor.getSelectedSensorPosition());
        this.dIntegral += (error*0.02);
        double derivative = (error-this.dprevious_error)/0.02;
        drcw = dkP* error + dkI * this.dIntegral + dkD * derivative;
    }

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

    @Override
    public void periodic() {
        // PIDLX();
        // PIDLY();
        // PIDS();    
        // PIDA();
        // PIDD();
        pose = odometry.update(getHeading(), getEncoderDistance(topLeftMotor), getEncoderDistance(topRightMotor));
    }
    
    @Override
    public void outputSmartdashboard() {
        // SmartDashboard.putNumber("Front Right Wheel", -topRightMotor.getMotorOutputPercent());
        // SmartDashboard.putNumber("Back Left Wheel", bottomLeftMotor.getMotorOutputPercent());
        // SmartDashboard.putNumber("Back Right Wheel", -bottomRightMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("Snakeye X", Lemonlight.getX());
        SmartDashboard.putNumber("Shooter Speed", Distance.ShooterSpeed());
        SmartDashboard.putNumber("Distance Traveled", getDistance());
        if (Limelight.getV() == 1.0)
            SmartDashboard.putBoolean("HasTarget", true);
        else
            SmartDashboard.putBoolean("HasTarget", false);
    }

    @Override
    public void zeroSensors() {
        gyro.reset();
        topLeftMotor.setSelectedSensorPosition(0);

    }

    @Override
    public void resetSubsystem() {
        
    }
 
    @Override
    public void testSubsystem() {

    }


}