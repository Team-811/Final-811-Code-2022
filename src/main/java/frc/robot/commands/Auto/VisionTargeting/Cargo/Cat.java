package frc.robot.commands.Auto.VisionTargeting.Cargo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Cat extends SequentialCommandGroup{
    private Drivetrain requiredSubsystem;
    
    public Cat(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
      addCommands(new CatSearch(requiredSubsystem), new CatFollow(requiredSubsystem));
    }
}
