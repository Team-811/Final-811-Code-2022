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
      drive.driveToJoy(-RobotContainer.driveController.leftStick.getY(), RobotContainer.driveController.leftStick.getX(), RobotContainer.driveController.rightStick.getX());  
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
