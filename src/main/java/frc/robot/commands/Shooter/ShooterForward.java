package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterForward extends CommandBase {
   private Shooter requiredSubsystem;

   public ShooterForward(Shooter m_SubsystemBase) {
       requiredSubsystem = m_SubsystemBase;
       addRequirements(requiredSubsystem);
   }
 
   @Override
   public void initialize() {}
 
   @Override
   public void execute() {
      double distance = GetDistance.Distance();
      double SHOOTER_SPEED = distance * 0.1; // placeholder 
      requiredSubsystem.shooterSpin(Constants.SHOOTER_SPEED);
}
 
   @Override
   public void end(boolean interrupted) {}
 
   @Override
   public boolean isFinished() {
     return true;
   }
 }