package frc.robot.commands.Drivetrain;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.BobXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Vision.LimelightFetch;


public class StrafeAndAimLeft extends CommandBase {
  
  private Drivetrain requiredSubsystem;
  private double left_command;
  private double right_command;

  static BobXboxController controller;
  
  
  public StrafeAndAimLeft(Drivetrain m_SubsystemBase, BobXboxController m_controller) {
    requiredSubsystem = m_SubsystemBase;
    controller = m_controller;
    addRequirements(requiredSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      double tx = LimelightFetch.getX();
      float Kp = 0.01f; 
      float min_command = 0.1f;
      float heading_error = (float)tx;
      float steering_adjust = 0.0f;
      left_command = 0;
      right_command = 0;
      if(tx > 0){
          steering_adjust = Kp*heading_error + min_command;
          left_command = Constants.STRAFE_AND_AIM_SPEED - steering_adjust;
          right_command = Constants.STRAFE_AND_AIM_SPEED + steering_adjust;

      }
      if (tx < 0){
          steering_adjust = Kp*heading_error - min_command;
          left_command = Constants.STRAFE_AND_AIM_SPEED - steering_adjust;
          right_command = Constants.STRAFE_AND_AIM_SPEED + steering_adjust;

      }
      requiredSubsystem.frontLeftForward(-left_command);
      requiredSubsystem.backLeftForward(left_command);
      requiredSubsystem.frontRightForward(right_command);
      requiredSubsystem.backRightForward(-right_command);
 
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
