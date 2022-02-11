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
        if (RobotContainer.driveController.leftStick.getX() < 0) {
            requiredSubsystem.turnLeft(Math.abs(RobotContainer.driveController.leftStick.getX()*0.6));
        }
        else if (RobotContainer.driveController.leftStick.getX() > 0) {
            requiredSubsystem.turnRight(Math.abs(RobotContainer.driveController.leftStick.getX()*0.6));
        }
/*
Something Worth Trying: find out values of joystick and do math to make turning speed adjustable
EX: requiredSubsystem.turnRight(RobotContainer.driveController.rightStick.getX() * .04 );
If joystick is 150 then the turn speed will be 6 (every 10 on the joystick is .4 on speed constraint)
It is a range between -1.0 and 1.0 with 0.0 being the joystick in resting position
*/

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

///////////////////////////////////////////////
// OLD CODE WHEN THE STUFF ABOVE DOESN'T WORK//
///////////////////////////////////////////////

// package frc.robot.commands.VisionTargeting.Hub;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Vision.LimelightFetch;
// import frc.robot.subsystems.Drivetrain;

// public class LimelightSearch extends CommandBase{
    
//     private Drivetrain requiredSubsystem;
//     private boolean found = false;

//     public LimelightSearch(Drivetrain m_SubsystemBase) {
//       requiredSubsystem = m_SubsystemBase;
//       addRequirements(requiredSubsystem);      
//     }
    
//     @Override
//     public void execute() {
//         double seen = LimelightFetch.getV();

//         if (seen == 1.0)
//             found = true;
//         else
//             requiredSubsystem.turnLeft(Constants.AIM_SPEED);

//     }
//     @Override
//     public void end(boolean interrupted) {
//         requiredSubsystem.stopRobot();
//     }
  
//     @Override
//     public boolean isFinished() {
//         if (found == true)
//             {
//                 found = false;
//                 return true;
//             }
//             else
//             {
//                 return false;
//             }
//     }
// }
