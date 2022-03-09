package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrivingCommand extends CommandBase {
  Drivetrain drive;
  public SwerveDrivingCommand(Drivetrain drivetrain) {
      addRequirements(drivetrain);
      drive = drivetrain;
  }

  @Override
  public void execute() {
      drive.adjustRotation(RobotContainer.driveController.rightStick.getX());
      drive.driveToJoy(RobotContainer.driveController.leftStick.getX(), RobotContainer.driveController.leftStick.getY());    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
