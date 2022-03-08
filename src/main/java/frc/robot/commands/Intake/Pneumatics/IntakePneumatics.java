package frc.robot.commands.Intake.Pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakePneumatics extends CommandBase {
  
  private Intake requiredSubsystem;

  public IntakePneumatics(Intake m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
  }
 
  @Override
  public void initialize() {}
 
  @Override
  public void execute() {
    if (!requiredSubsystem.getState()){ 
      requiredSubsystem.extendIntake();
      requiredSubsystem.toggleState();
    }
  }
 
  @Override
  public void end(boolean interrupted) {}
 
  @Override
  public boolean isFinished() {
    if (requiredSubsystem.getState())
      return true;
    else 
      return false;
  }
}