package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup{
    private Shooter requiredSubsystem;
    
    public Shoot(Shooter m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
      addCommands(new ShooterForward(requiredSubsystem));
    }
}
