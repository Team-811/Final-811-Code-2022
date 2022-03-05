package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRelease extends CommandBase {
   
  private Intake requiredSubsystem;

  public IntakeRelease(Intake m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
  }
 
  @Override
  public void initialize() {}
 
  @Override
  public void execute() {
      requiredSubsystem.intakeStop();
      requiredSubsystem.backStop();
      requiredSubsystem.retractIntake();
      requiredSubsystem.toggleState();
  }
 
   @Override
   public void end(boolean interrupted) {}
 
   @Override
   public boolean isFinished() {
     return true;
   }
 }