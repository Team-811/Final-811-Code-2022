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

  public BackwardsTwoBallAuto(Drivetrain drive, Intake intake, Shooter shoot) {
    super(
      new InstantCommand(() -> intake.extendIntake()), //Extend Intake
      new ParallelDeadlineGroup( //Drive Backwards for 3 seconds (placeholder)
        new WaitCommand(3),                                   
        new InstantCommand(() -> drive.driveBackwards(0.3))
      ),   
      new ParallelDeadlineGroup( //Reverse Intake and Shooter
        new WaitCommand(0.2),                                        
        new InstantCommand(() -> shoot.shooterSpin(-0.3), shoot),
        new InstantCommand(() -> intake.backSpin(-0.2), intake)
      ),
      new ParallelDeadlineGroup( //Ramp up shooter
        new WaitCommand(1.5),                                                        
        new InstantCommand(() -> shoot.shooterSpin((Distance.get() * .038)), shoot)
      ),
      new ParallelDeadlineGroup( //Storage motor into shooter for shot
        new WaitCommand(1.5),                                                         
        new IntakeForward(intake),
        new InstantCommand(() -> shoot.shooterSpin((Distance.get() * .038)), shoot)
      ),
      new ParallelDeadlineGroup( //Disable shooter and intake                                     
        new ShooterStop(shoot),
        new InstantCommand(() -> intake.intakeSpin(0), intake),
        new InstantCommand(() -> intake.backSpin(0), intake)
      ),
      new InstantCommand(() -> drive.aSetpoint(180)), //Set Gyro point to 180 degrees (placeholder)
      new ParallelDeadlineGroup( //Use Gyro and PID to lock to 180 degrees
        new WaitCommand(2),
        new InstantCommand(() -> drive.driveAuto())
      ),
      new InstantCommand(() -> drive.aSetpoint(0)), //Set Gyro point to 0 degrees (placeholder)
      new ParallelDeadlineGroup(  //Drive and Pick up second ball
        new WaitCommand(2),
        new InstantCommand(() -> drive.driveForward(0.3)),
        new InstantCommand(() -> intake.intakeSpin(0.35)),
        new InstantCommand(() -> intake.backSpin(0.3))
      ),
      new ParallelDeadlineGroup( //Disable shooter and intake 
        new InstantCommand(() -> intake.intakeSpin(0), intake),
        new InstantCommand(() -> intake.backSpin(0), intake)
      ),
      new ParallelDeadlineGroup(//Use Gyro and PID to lock to 0 degrees
        new WaitCommand(2),
        new InstantCommand(() -> drive.driveAuto())
      ),
      new ParallelDeadlineGroup( //Reverse Intake and Shooter
        new WaitCommand(0.2),                                        
        new InstantCommand(() -> shoot.shooterSpin(-0.3), shoot),
        new InstantCommand(() -> intake.backSpin(-0.2), intake)
      ),
      new ParallelDeadlineGroup( //Ramp up shooter
        new WaitCommand(1.5),                                                        
        new InstantCommand(() -> shoot.shooterSpin((Distance.get() * .038)), shoot)
      ),
      new ParallelDeadlineGroup( //Storage motor into shooter for shot
        new WaitCommand(1.5),                                                         
        new IntakeForward(intake),
        new InstantCommand(() -> shoot.shooterSpin((Distance.get() * .038)), shoot)
      ),
      new ParallelDeadlineGroup( //Disable shooter and intake                                     
        new ShooterStop(shoot),
        new InstantCommand(() -> intake.intakeSpin(0), intake),
        new InstantCommand(() -> intake.backSpin(0), intake)
      ));
  }
}
