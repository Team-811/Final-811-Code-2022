package frc.robot.commands.Auto.VisionTargeting.Hub;

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
        // float Kp = 0.02f; 
        // float min_command = 0.05f;
        // float heading_error = (float)feet;
        // float steering_adjust = 0.0f;
        // left_command = 0;
        // right_command = 0;
        // if(feet > 9){
        //     steering_adjust = Kp*heading_error + min_command;
        //     left_command += steering_adjust;
        //     right_command += steering_adjust;
        //     requiredSubsystem.leftWheelsForward(left_command);
        //     requiredSubsystem.rightWheelsForward(right_command);
        // }
        // if (feet < 9){
        //     steering_adjust = Kp*heading_error + min_command;
        //     left_command += steering_adjust;
        //     right_command += steering_adjust;
        //     requiredSubsystem.leftWheelsForward(-left_command);
        //     requiredSubsystem.rightWheelsForward(-right_command);
        // }
        if(feet > 9 && feet != 0)
        {
            requiredSubsystem.rightWheelsForward(Constants.CAT_DRIVE_SPEED);
            requiredSubsystem.leftWheelsForward(Constants.CAT_DRIVE_SPEED);
        }
        else 
        {
            requiredSubsystem.rightWheelsForward(-Constants.CAT_DRIVE_SPEED);
            requiredSubsystem.leftWheelsForward(-Constants.CAT_DRIVE_SPEED);
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