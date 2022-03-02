package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.VisionProcessing.Distance;
import frc.robot.commands.Intake.Motors.IntakeForward;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {
  
  private static double speed = Distance.get() * .05;

  public Shoot(Shooter shoot, Intake intake) {
    super(
      new ParallelDeadlineGroup(
        new WaitCommand(0.2),
        new InstantCommand(() -> shoot.shooterSpin(-0.3), shoot),
        new InstantCommand(() -> intake.backSpin(-0.2), intake)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1.5),
        new InstantCommand(() -> shoot.shooterSpin(speed), shoot)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1.5),
        new IntakeForward(intake),
        new InstantCommand(() -> shoot.shooterSpin(speed), shoot)
      ),
      new ShooterStop(shoot),
      new InstantCommand(() -> intake.intakeSpin(0), intake),
      new InstantCommand(() -> intake.backSpin(0), intake)
    );
  }
}
