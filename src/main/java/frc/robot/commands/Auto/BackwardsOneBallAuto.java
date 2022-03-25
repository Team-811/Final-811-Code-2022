package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BackwardsOneBallAuto extends SequentialCommandGroup {

  public BackwardsOneBallAuto(Drivetrain drive, Intake intake, Shooter shoot) {
    super(
      new InstantCommand(() -> intake.extendIntake(), intake), //Extend Intake
      new ParallelDeadlineGroup( //Drive Backwards 
        new WaitCommand(0.1),                                   
        new InstantCommand(() -> drive.driveBackwards(0.2), drive)
      ),
      new ParallelDeadlineGroup( //Reverse Intake and Shooter
        new WaitCommand(0.75),                                        
        new InstantCommand(() -> shoot.shooterSpin(-0.3), shoot),
        new InstantCommand(() -> intake.backSpin(-0.2), intake)
      ),
      new ParallelDeadlineGroup( //Ramp up shooter
        new WaitCommand(1.5),                                                        
        new InstantCommand(() -> shoot.shooterSpin(.75), shoot)
      ),
      new ParallelDeadlineGroup( //Storage motor into shooter for shot
        new WaitCommand(2),                                                         
        new InstantCommand(() ->intake.backSpin(0.5), intake),
        new InstantCommand(() -> shoot.shooterSpin(.75), shoot)
      ),
       //Disable shooter and intake                                     
      new InstantCommand(() -> shoot.shooterSpin(0), shoot),
      new InstantCommand(() -> {intake.intakeSpin(0);
                                intake.backSpin(0);}, intake),
    new InstantCommand(() -> intake.retractIntake(), intake),
    new InstantCommand(()->drive.driveForward(0), drive)
    );
  }
}
