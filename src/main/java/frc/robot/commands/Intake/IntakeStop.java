package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeStop extends CommandBase {
   private Intake requiredSubsystem;

   public IntakeStop(Intake m_SubsystemBase) {
       requiredSubsystem = m_SubsystemBase;
       addRequirements(requiredSubsystem);
   }
 
   @Override
   public void initialize() {}
 
   @Override
   public void execute() {
       requiredSubsystem.intakeStop();
       requiredSubsystem.backStop();

}
 
   @Override
   public void end(boolean interrupted) {}
 
   @Override
   public boolean isFinished() {
     return false;
   }
 }