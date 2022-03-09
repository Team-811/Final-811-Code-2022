package frc.robot.VisionProcessing.VisionTargeting.Hub;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class LimelightAim extends SequentialCommandGroup{
    private Drivetrain requiredSubsystem;
    
    public LimelightAim(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
      addCommands(new LimelightAimX(requiredSubsystem));
    }
}
