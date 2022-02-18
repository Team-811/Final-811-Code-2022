package frc.robot.commands.VisionTargeting.Hub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Vision.LimelightFetch;
import frc.robot.subsystems.Drivetrain;

public class LimelightSearch extends CommandBase{
    
    private Drivetrain requiredSubsystem;
    private boolean found = false;


    public LimelightSearch(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);      
    }
    
    @Override
    public void execute() {
        double seen = LimelightFetch.getV();
        if (seen == 1.0)
            found = true;
        requiredSubsystem.turnRight(Constants.AIM_SPEED);
        // if (RobotContainer.driveController.rightStick.getX() < 0) {
        //     requiredSubsystem.turnLeft(Math.abs(RobotContainer.driveController.rightStick.getX()*0.6));
        // }
        // else if (RobotContainer.driveController.rightStick.getX() > 0) {
        //     requiredSubsystem.turnRight(Math.abs(RobotContainer.driveController.rightStick.getX()*0.6));
        // }

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
