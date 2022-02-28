package frc.robot.commands.Auto.VisionTargeting.Cargo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.VisionProcessing.Lemonlight;
import frc.robot.subsystems.Drivetrain;

public class CatSearch extends CommandBase{
    
    private Drivetrain requiredSubsystem;
    private boolean found = false;

    public CatSearch(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);      
    }
    
    @Override
    public void execute() {
        boolean seen = Lemonlight.getV();
        if (seen == true)
            found = true;
        else
            requiredSubsystem.turnLeft(Constants.OBJECT_AIM_SPEED);

    }
    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        if (found == true)
            {
                found = false;
                return true;
            }
            else
            {
                return false;
            }
    }
}
