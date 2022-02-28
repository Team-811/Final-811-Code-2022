package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class SimpleAuto extends ParallelDeadlineGroup {
  
  public SimpleAuto(Drivetrain subsystem) {
    super(new WaitCommand(1), new InstantCommand(()->subsystem.driveForward(0.3), subsystem));
  }
}
