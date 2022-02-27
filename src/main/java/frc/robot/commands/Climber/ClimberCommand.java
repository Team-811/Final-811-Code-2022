// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends CommandBase {
  Climber m_subsystem;
  /** Creates a new ClimberCommand. */
  public ClimberCommand(Climber climber) {
    m_subsystem = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operatorController.rightBumper.get()){
      m_subsystem.rightWinchRun(-Constants.WINCH_SPEED);
    }else if(RobotContainer.operatorController.leftBumper.get()){
      m_subsystem.rightWinchRun(Constants.WINCH_SPEED);
    }else{
      m_subsystem.rightWinchRun(0);
    }
    if(RobotContainer.operatorController.rightTriggerButton.get()){
      m_subsystem.leftWinchRun(-Constants.WINCH_SPEED);
    }else if(RobotContainer.operatorController.leftTriggerButton.get()){
      m_subsystem.leftWinchRun(Constants.WINCH_SPEED);
    }else{
      m_subsystem.rightWinchRun(0);
    }
    // m_subsystem.leftArm(RobotContainer.operatorController.leftStick.getY());
    // m_subsystem.rightArm(RobotContainer.operatorController.rightStick.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
