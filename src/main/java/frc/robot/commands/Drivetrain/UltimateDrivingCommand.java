// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Vision.LimelightFetch;
import frc.robot.commands.Shooter.GetDistance;
import frc.robot.subsystems.Drivetrain;

public class UltimateDrivingCommand extends CommandBase {
  /** Creates a new UltimateDrivingCommand. */
  Drivetrain drive;
  public UltimateDrivingCommand(Drivetrain drivetrain) {
      addRequirements(drivetrain);
      drive = drivetrain;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double tx = LimelightFetch.getX();
    float Kp = 0.02f; 
    float min_command = 0.05f;
    float heading_error = (float)tx;
    float steering_adjust = 0.0f;
    if(tx > 0){
      steering_adjust = Kp*heading_error - min_command;
    }
    if (tx < 0){
      steering_adjust = Kp*heading_error + min_command;
    }
    if(RobotContainer.driveController.xButton.get()==false){
      steering_adjust = 0;
    }
    double forwardAdjust=0;
    double feet  = GetDistance.Distance();
    if(RobotContainer.driveController.yButton.get()){
      if(feet > 9 && feet != 0)
      {
        forwardAdjust = 0.1;
      }
      else
      {
        forwardAdjust = -0.1;
      }
    }else{
      forwardAdjust = 0;
    }
    if(RobotContainer.driveController.rightTriggerButton.get()){
      //Strafe Right
      drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY() + forwardAdjust,
      RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX() + steering_adjust, Constants.STRAFE_SPEED, Constants.STRAFE_SPEED, -Constants.STRAFE_SPEED, -Constants.STRAFE_SPEED);
    }else if(RobotContainer.driveController.leftTriggerButton.get()){
      //Strafe Left
      drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY() + forwardAdjust,
      RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX() + steering_adjust, -Constants.STRAFE_SPEED, -Constants.STRAFE_SPEED, Constants.STRAFE_SPEED, Constants.STRAFE_SPEED);
    }else{
      //Classic Arcade Drive
      drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY() + forwardAdjust,
      RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX() + steering_adjust);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
