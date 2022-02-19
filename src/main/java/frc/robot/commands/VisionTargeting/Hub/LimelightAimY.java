package frc.robot.commands.VisionTargeting.Hub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.LimelightFetch;
import frc.robot.commands.Shooter.GetDistance;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimY extends CommandBase {

    private Drivetrain requiredSubsystem;
    private double left_command;
    private double right_command;
    private boolean exit = false;
  
    public LimelightAimY(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
    }
  
    @Override
    public void execute() {
        if (LimelightFetch.getV() != 1.0) {
            exit = true;
        }
        double feet  = GetDistance.Distance();
        float Kp = 0.02f; 
        float min_command = 0.05f;
        float heading_error = (float)feet;
        float steering_adjust = 0.0f;
        left_command = 0;
        right_command = 0;
        if(feet > 10){
            steering_adjust = Kp*heading_error - min_command;
        }
        if (feet < 11){
            steering_adjust = Kp*heading_error + min_command;
        }
        left_command += steering_adjust;
        right_command += steering_adjust;
        requiredSubsystem.leftWheelsForward(left_command);
        requiredSubsystem.rightWheelsForward(right_command);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        if (exit == true){
            exit = false;
            return true;
        }
        double feet = GetDistance.Distance();
        if(feet >= 10.0 && feet <= 11.0 && feet !=0.0)
        {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (feet >= 10.0 && feet <= 11.0 && feet !=0.0)
                return true;
        }
        return false;    
    }
}