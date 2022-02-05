package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.Auto;

public class AutoForward extends CommandBase {
   private Auto requiredSubsystem;

   private boolean autoOn = true; //I am fully aware that this is incorrect 

   public AutoForward(Auto m_SubsystemBase) {
       requiredSubsystem = m_SubsystemBase;
       addRequirements(requiredSubsystem);
   }
 
   @Override
   public void initialize() {}
 
   @Override
   public void execute() {
    if(autoOn == true){ // we need a way to tell the robot that auto exists]
        
    }else {

    }
}
 
   @Override
   public void end(boolean interrupted) {}
 
   @Override
   public boolean isFinished() {
     return false;
   }
 }
 
