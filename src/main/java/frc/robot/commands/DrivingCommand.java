// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.NetworkTables.LimelightFetch;
import frc.robot.NetworkTables.SnakeEyesFetch;
import frc.robot.subsystems.Drivetrain;

public class DrivingCommand extends CommandBase {
  Drivetrain drive;
  public DrivingCommand(Drivetrain drivetrain) {
      addRequirements(drivetrain);
      drive = drivetrain;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double tx = LimelightFetch.getX();
    double bx = SnakeEyesFetch.getX();
    float Kp = 0.04f; 
    float min_command = 0.1f; //Last value that worked: 0.075
    float heading_error_target = (float)tx;
    float heading_error_object = (float)bx;
    float steering_adjust = 0.0f;
    //Hold X for target aiming
    if (RobotContainer.driveController.xButton.get() == true)
        if(tx > 0){
            steering_adjust = Kp*heading_error_target - min_command;
        }
        if (tx < 0){
            steering_adjust = Kp*heading_error_target + min_command;
    }
    //Hold Y for object aiming
    else if (RobotContainer.driveController.yButton.get() == true)
        if(tx > 0){
            steering_adjust = Kp*heading_error_object - min_command;
        }
        if (tx < 0){
            steering_adjust = Kp*heading_error_object + min_command;
    }
    //No aiming
    else {
      steering_adjust = 0;
    }
    
    if(RobotContainer.driveController.rightTriggerButton.get()){
      //Strafe Right
      drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY(), /*+ forwardAdjust,*/   //Left Stick
                            RobotContainer.driveController.rightStick.getY(),                       //Right Stick
                            RobotContainer.driveController.rightStick.getX() + steering_adjust,     //Rotation
                            Constants.STRAFE_SPEED,                                                 //Front Left
                            Constants.STRAFE_SPEED,                                                 //Front Right
                            -Constants.STRAFE_SPEED,                                                //Back Left 
                            -Constants.STRAFE_SPEED);                                               //Back Right
    }else if(RobotContainer.driveController.leftTriggerButton.get()){
      //Strafe Left
      drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY(), /*+ forwardAdjust,*/  //Left Stick
                            RobotContainer.driveController.rightStick.getY(),                       //Right Stick
                            RobotContainer.driveController.rightStick.getX() + steering_adjust,     //Rotation
                            -Constants.STRAFE_SPEED,                                                //Front Left
                            -Constants.STRAFE_SPEED,                                                //Front Right
                            Constants.STRAFE_SPEED,                                                 //Back Left
                            Constants.STRAFE_SPEED);                                                //Back Right
    }else{
      //Classic Arcade Drive
      drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY(), /*+ forwardAdjust,*/   //Left Stick
                            RobotContainer.driveController.rightStick.getY(),                       //Right Stick
                            RobotContainer.driveController.rightStick.getX() + steering_adjust);    //Rotation
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
