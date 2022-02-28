package frc.robot.commands.Intake.Motors;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeReverse extends CommandBase {
   
  private Intake requiredSubsystem;

  public IntakeReverse(Intake m_SubsystemBase, Shooter shoot) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
   }
 
  @Override
  public void initialize() {}
 
  @Override
  public void execute() {
      requiredSubsystem.backSpin(-0.2);
      requiredSubsystem.intakeSpin(-Constants.REAL_INTAKE_SPEED);
  }
 
  @Override
  public void end(boolean interrupted) {}
 
  @Override
  public boolean isFinished() {
      return true;
  }
}