package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.Motors.IntakeMotorsReverse;
import frc.robot.commands.Intake.Pneumatics.IntakePneumatics;
import frc.robot.subsystems.Intake;

public class IntakeReversePress extends SequentialCommandGroup{
    private Intake requiredSubsystem;
    
    public IntakeReversePress(Intake m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
      addCommands(new IntakePneumatics(requiredSubsystem), new IntakeMotorsReverse(requiredSubsystem));
    }
}
