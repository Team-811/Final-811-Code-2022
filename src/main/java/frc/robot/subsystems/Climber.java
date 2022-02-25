// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase implements ISubsystem {
  private TalonSRX leftWinch;
  private TalonSRX leftArm;
  //Right Motors
  private TalonSRX rightWinch;
  private TalonSRX rightArm;

  /** Creates a new Climber. */
  public Climber() {}

  public void leftArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    leftArm.set(ControlMode.PercentOutput, speed);
  }

  public void rightArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    rightArm.set(ControlMode.PercentOutput, speed);
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
