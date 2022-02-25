package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.annotation.JsonAppend.Prop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase implements ISubsystem{
  //Left Motors
  private TalonSRX leftWinch;
  private TalonSRX leftArm;
  //Right Motors
  private TalonSRX rightWinch;
  private TalonSRX rightArm;
  

  private double leftEncoderValue; 
  private double rightEncoderValue;
  /** Creates a new Climber. */
  public Climber() {
    leftWinch = new TalonSRX(RobotMap.CLIMBER_WINCH_LEFT);
    leftArm = new TalonSRX(RobotMap.CLIMBER_ARM_LEFT);
    rightWinch = new TalonSRX(RobotMap.CLIMBER_WINCH_RIGHT);
    rightArm = new TalonSRX(RobotMap.CLIMBER_ARM_RIGHT);
    lErrorSum = 0;
    LastTimeStamp = Timer.getFPGATimestamp();
    lLastError = 0;
    rErrorSum = 0;
    rLastError =0;
  }
  private double LastTimeStamp;
  private double ProportionalErrorMargin = 1;

  private double kLP = 0.01;
  private double kLI = 0.01;
  private double kLD = 0.01;
  private double lSetpoint;
  private double lErrorSum;
  private double lLastError;
  public void leftArmAngle(double angle){
    lSetpoint = angle;
  }
  private double kRP;
  private double kRI;
  private double kRD;
  private double rSetpoint;
  private double rErrorSum;
  private double rLastError;
  public void rightArmAngle(double angle){
    rSetpoint = angle;
  }


  @Override
  public void periodic() {
    leftEncoderValue = 1; //Get encoder values in degrees
    rightEncoderValue = 1;

    //Left Calculations
    double leftError = lSetpoint - leftEncoderValue;
    double ldt = Timer.getFPGATimestamp() - LastTimeStamp;
    if(Math.abs(leftError)<ProportionalErrorMargin)
      lErrorSum += leftError * ldt;
    double lErrorRate = (leftError - lLastError)/ldt;
    double leftOutputSpeed = kLP * leftError + kLI * lErrorSum + kLD * lErrorRate;
    //Right Calculations
    double rightError = rSetpoint - rightEncoderValue;
    double rdt = Timer.getFPGATimestamp()  -LastTimeStamp;
    if(Math.abs(rightError)<ProportionalErrorMargin)
      rErrorSum += rightError * rdt;
    double rErrorRate = (rightError - rLastError)/rdt;
    double rightOutputSpeed = kRP * rightError + kRI * rErrorSum + kRD * rErrorRate;
    //Motor control
    leftWinch.set(ControlMode.PercentOutput, leftOutputSpeed);
    rightWinch.set(ControlMode.PercentOutput, rightOutputSpeed);
    //Time Update
    LastTimeStamp= Timer.getFPGATimestamp();
    lLastError = leftError;
    rLastError = rightError;
  }

  @Override
  public void outputSmartdashboard() {
    
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
