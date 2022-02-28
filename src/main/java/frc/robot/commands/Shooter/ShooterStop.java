package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterStop extends CommandBase {
  private Shooter requiredSubsystem;

  public ShooterStop(Shooter m_SubsystemBase) {
    requiredSubsystem = m_SubsystemBase;
    addRequirements(requiredSubsystem);
  }
 
  @Override
  public void initialize() {}
 
  @Override
  public void execute() {
    requiredSubsystem.shooterStop();
  }
 
  @Override
  public void end(boolean interrupted) {}
 
  @Override
  public boolean isFinished() {
    return true;
  }
}