// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class SimpleAuto extends ParallelDeadlineGroup {
  public SimpleAuto(Drivetrain subsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    super(new WaitCommand(1), new InstantCommand(()->subsystem.driveForward(0.3), subsystem));


  }

}
