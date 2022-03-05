package frc.robot.commands.Intake.Pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeToggle extends CommandBase{

    private Intake requiredSubsystem;
    
    public IntakeToggle(Intake m_SubsystemBase) {
        requiredSubsystem = m_SubsystemBase;
        addRequirements(requiredSubsystem);
    }
 
    @Override
    public void initialize() {}
 
    @Override
    public void execute() {
        if (requiredSubsystem.toggleState())
            requiredSubsystem.retractIntake();
        else
            requiredSubsystem.extendIntake();
    }
 
   @Override
   public void end(boolean interrupted) {}
 
   @Override
   public boolean isFinished() {
     return true;
   }    
}
