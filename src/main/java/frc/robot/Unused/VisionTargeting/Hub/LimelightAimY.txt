package frc.robot.VisionProcessing.VisionTargeting.Hub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.VisionProcessing.Distance;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimY extends CommandBase {
    private Drivetrain requiredSubsystem;
    // private double left_command;
    // private double right_command;

   public LimelightAimY(Drivetrain m_SubsystemBase) {
       requiredSubsystem = m_SubsystemBase;
       addRequirements(requiredSubsystem);
   }
 
   @Override
   public void initialize() {}
 
   @Override
    public void execute() {
        double feet  = Distance.get();
        if(feet > 9 && feet != 0)
        {
            requiredSubsystem.rightWheelsForward(Constants.CAT_DRIVE_SPEED);
            requiredSubsystem.leftWheelsForward(Constants.CAT_DRIVE_SPEED);
        }
        else 
        {
            requiredSubsystem.rightWheelsBackwards(Constants.CAT_DRIVE_SPEED);
            requiredSubsystem.leftWheelsBackwards(Constants.CAT_DRIVE_SPEED);
        }
    }    

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        double feet = Distance.get();
        if(feet >= 8.5 && feet <= 9.5 && feet !=0.0)
        {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (feet >= 8.5 && feet <= 9.5 && feet !=0.0)
                return true;
        }
        return false;    
    }
}