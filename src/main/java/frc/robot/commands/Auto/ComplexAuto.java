package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.VisionTargeting.Cargo.CatFollow;
import frc.robot.commands.Auto.VisionTargeting.Cargo.CatSearch;
import frc.robot.commands.Auto.VisionTargeting.Hub.LimelightAim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ComplexAuto extends SequentialCommandGroup {

  public ComplexAuto(Drivetrain drive, Intake intake, Shooter shooter) {
    super(
      new InstantCommand(() -> intake.extendIntake()),
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        new InstantCommand(() -> drive.driveBackwards(0.3))
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.8), 
        new InstantCommand(() -> shooter.shooterSpin(1))
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        new InstantCommand(() -> {
          intake.backSpin(0.1);
          shooter.shooterSpin(0.1);
        })
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(2), 
        new CatSearch(drive)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(8), 
        new CatFollow(drive),
        new InstantCommand(() -> intake.intakeSpin(0.1)),
        new InstantCommand(() -> intake.backSpin(0.1))
      ),
      new ParallelCommandGroup(
        new WaitCommand(5),
        new LimelightAim(drive)
      ),
      new ParallelCommandGroup(
        new WaitCommand(0.5),
        new InstantCommand(() -> intake.backSpin(0.1))
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.8), 
        new InstantCommand(() -> shooter.shooterSpin(1))
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        new InstantCommand(() -> {
          intake.backSpin(0.1);
          shooter.shooterSpin(0.1);
        })
      )
    );
  }
}
