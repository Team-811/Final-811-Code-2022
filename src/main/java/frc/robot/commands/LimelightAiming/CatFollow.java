package frc.robot.commands.LimelightAiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
// import frc.robot.Vision.LimelightFetch;
import frc.robot.Vision.SnakeEyesFetchTest;
import frc.robot.subsystems.Drivetrain;

public class CatFollow extends CommandBase {

    private Drivetrain requiredSubsystem;
    private double left_command;
    private double right_command;
  
    public CatFollow(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
    }
  
    @Override
    public void execute() {
        double tx = SnakeEyesFetchTest.getX();
        float Kp = 0.03f; 
        float min_command = 0.03f;
        float heading_error = (float)tx;
        float steering_adjust = 0.0f;
        left_command = 0;
        right_command = 0;
        if(tx > 1.0){
            steering_adjust = Kp*heading_error - min_command;
        }
        if (tx < 1.0){
            steering_adjust = Kp*heading_error + min_command;
        }
        left_command = left_command + steering_adjust + Constants.CAT_DRIVE_SPEED;
        right_command = right_command - steering_adjust + Constants.CAT_DRIVE_SPEED;
        requiredSubsystem.leftWheelsForward(left_command);
        requiredSubsystem.rightWheelsForward(right_command);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        double x = SnakeEyesFetchTest.getX();
        if(x >= -20.0 && x <= 20.0)
        {
             try {
             Thread.sleep(50);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }
            if (x >= -20.0 && x <= 20.0)
                return true;
        }
        if (SnakeEyesFetchTest.getV() != 1.0) 
            return true;
        return false;    
    }
}