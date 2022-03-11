package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DrivingCommand extends CommandBase {
  Drivetrain drive;
  public DrivingCommand(Drivetrain drivetrain) {
      addRequirements(drivetrain);
      drive = drivetrain;
  }

  @Override
  public void execute() {
    //Strafe Right
    if(RobotContainer.driveController.rightTriggerButton.get()){
      drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY(), RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX(), Constants.STRAFE_SPEED, Constants.STRAFE_SPEED, -Constants.STRAFE_SPEED, -Constants.STRAFE_SPEED);                              
    //Strafe Left
    } else if (RobotContainer.driveController.leftTriggerButton.get()){drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY(), RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX(), -Constants.STRAFE_SPEED, -Constants.STRAFE_SPEED, Constants.STRAFE_SPEED, Constants.STRAFE_SPEED);                               
    //Normal Drive
    } else {
      drive.driveWithMisery(-RobotContainer.driveController.leftStick.getY(), RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX());    
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}