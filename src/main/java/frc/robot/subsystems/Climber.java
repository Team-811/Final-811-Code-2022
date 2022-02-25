// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase implements ISubsystem {
  private CANVenom leftWinch;
  private TalonSRX leftArm;
  //Right Motors
  private CANVenom rightWinch;
  private TalonSRX rightArm;
  
  // private DigitalInput rightLimit = new DigitalInput(0);
  // private DigitalInput leftLimit = new DigitalInput(0);

  /** Creates a new Climber. */
  public Climber() {
    //define the motors
  }

  public void leftArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    leftArm.set(ControlMode.PercentOutput, speed);
  }

  public void rightArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    rightArm.set(ControlMode.PercentOutput, speed);
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
