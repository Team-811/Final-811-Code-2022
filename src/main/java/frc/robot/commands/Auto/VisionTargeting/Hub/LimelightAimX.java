package frc.robot.commands.Auto.VisionTargeting.Hub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionProcessing.Limelight;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimX extends CommandBase {

    private Drivetrain requiredSubsystem;
    private double left_command;
    private double right_command;
    private boolean exit = false;
  
    public LimelightAimX(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
    }
  
    @Override
    public void execute() {
        if (Limelight.getV() != 1.0) {
            exit = true;
        }
        double tx = Limelight.getX();
        float Kp = 0.02f; 
        float min_command = 0.05f;
        float heading_error = (float)tx;
        float steering_adjust = 0.0f;
        left_command = 0;
        right_command = 0;
        if(tx > 0){
            steering_adjust = Kp*heading_error - min_command;
        }
        if (tx < 0){
            steering_adjust = Kp*heading_error + min_command;
        }
        left_command += steering_adjust;
        right_command -= steering_adjust;
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
        double x = Limelight.getX();
        if(x >= -2.0 && x <= 2.0 && x !=0.0)
        {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (x >= -3.0 && x <= 3.0 && x !=0.0)
                return true;
        }
        return false;    
    }
}