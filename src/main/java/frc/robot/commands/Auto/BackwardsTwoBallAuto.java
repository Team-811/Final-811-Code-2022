package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.VisionProcessing.Distance;
import frc.robot.commands.Intake.Motors.IntakeForward;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BackwardsTwoBallAuto extends ParallelDeadlineGroup {
  
  private static double speed = Distance.get() * .038;

  public BackwardsTwoBallAuto(Drivetrain drive, Intake intake, Shooter shoot) {
    super(
      new InstantCommand(() -> intake.extendIntake()),
      new WaitCommand(3),
      new InstantCommand(() -> drive.driveBackwards(0.3)),
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
      new ParallelDeadlineGroup(
        new WaitCommand(0.25),
        new ShooterStop(shoot),
        new InstantCommand(() -> intake.intakeSpin(0), intake),
        new InstantCommand(() -> intake.backSpin(0), intake)
      ));
  }
}
