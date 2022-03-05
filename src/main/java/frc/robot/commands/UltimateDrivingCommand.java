package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class UltimateDrivingCommand extends CommandBase {
    Drivetrain drive;


    public UltimateDrivingCommand(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        drive = drivetrain;
    }

    @Override
    public void execute() {
        drive.driveToJoy(RobotContainer.driveController.getX(), RobotContainer.driveController.getY());
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
  }