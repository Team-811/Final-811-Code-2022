// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndShootSlow extends SequentialCommandGroup {
  /** Creates a new IntakeAndShoot. */
  public IntakeAndShootSlow(Shooter shoot, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new ParallelDeadlineGroup(
        new WaitCommand(0.3),
        new InstantCommand(() -> intake.backSpin(-Constants.INTAKE_SPEED))
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> shoot.shooterSpin(0.5))
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> shoot.shooterSpin(Constants.SHOOTER_SPEED/2 + 0.05)),
        new InstantCommand(() -> intake.backSpin(Constants.INTAKE_SPEED))
      )
    );
  }
}
