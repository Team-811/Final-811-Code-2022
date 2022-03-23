package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends CommandBase {
  
  Climber m_subsystem;

  public ClimberCommand(Climber climber) {
    m_subsystem = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_subsystem.redArm(RobotContainer.operatorController.leftStick.getY());
    m_subsystem.yellowArm(RobotContainer.operatorController.rightStick.getY());

    if(RobotContainer.operatorController.leftBumper.get()){
      m_subsystem.redWinchRun(-Constants.WINCH_SPEED);
    } else if (RobotContainer.operatorController.leftTriggerButton.get()){
      m_subsystem.redWinchRun(Constants.WINCH_SPEED);
    } else {
      m_subsystem.redWinchRun(0);} 
    
    if(RobotContainer.operatorController.rightBumper.get()){
      m_subsystem.yellowWinchRun(-Constants.WINCH_SPEED);
    } else if (RobotContainer.operatorController.rightTriggerButton.get()){
      m_subsystem.yellowWinchRun(Constants.WINCH_SPEED); 
    } else {
      m_subsystem.yellowWinchRun(0);}
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
