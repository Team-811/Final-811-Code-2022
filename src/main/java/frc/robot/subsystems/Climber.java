// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase implements ISubsystem {
  private CANVenom leftWinch;
  // private TalonSRX leftArm;
  //Right Motors
  private CANVenom rightWinch;
  // private TalonSRX rightArm;
  
  // private DigitalInput rightLimit = new DigitalInput(0);
  // private DigitalInput leftLimit = new DigitalInput(0);

  /** Creates a new Climber. */
  public Climber() {
    leftWinch = new CANVenom(RobotMap.CLIMBER_WINCH_LEFT);
    // leftArm = new TalonSRX(RobotMap.CLIMBER_ARM_LEFT);
    rightWinch = new CANVenom(RobotMap.CLIMBER_WINCH_RIGHT);
    // rightArm = new TalonSRX(RobotMap.CLIMBER_ARM_RIGHT);
  }

  public void leftArm(double joystick){
    @SuppressWarnings("unused")
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    // leftArm.set(ControlMode.PercentOutput, speed);
  }

  public void rightArm(double joystick){
    @SuppressWarnings("unused")
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    // rightArm.set(ControlMode.PercentOutput, speed);
  }

  public void rightWinchRun(double speed){
    if(speed < 0){
      // if(!rightLimit.get()){
        rightWinch.set(speed);
      // }
    }else{
      rightWinch.set(speed);
    }
  }
    public void leftWinchRun(double speed){
      if(speed < 0){
        // if(!leftLimit.get()){
          leftWinch.set(speed);
        // }
      }else{
        leftWinch.set(speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
