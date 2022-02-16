package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterReverse extends CommandBase {
   private Shooter requiredSubsystem;

   public ShooterReverse(Shooter m_SubsystemBase) {
       requiredSubsystem = m_SubsystemBase;
       addRequirements(requiredSubsystem);
   }
 
   @Override
   public void initialize() {}
 
   @Override
   public void execute() {
       requiredSubsystem.shooterSpin(-Constants.SHOOTER_SPEED);

}
 
   @Override
   public void end(boolean interrupted) {}
 
   @Override
   public boolean isFinished() {
     return true;
   }
 }