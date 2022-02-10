package frc.robot.commands.VisionTargeting.Hub;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Vision.LimelightFetch;
import frc.robot.subsystems.Drivetrain;

public class LimelightSearch extends CommandBase{
    
    private Drivetrain requiredSubsystem;
    private boolean found = false;
    private TalonSRX topLeftMotor;
    private TalonSRX topRightMotor;
    private TalonSRX bottomLeftMotor;
    private TalonSRX bottomRightMotor;
    private double SpeedScale = Constants.DRIVETRAIN_SPEED_SCALE;

    public LimelightSearch(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);      
    }
    
    @Override
    public void execute() {
        double seen = LimelightFetch.getV();
        double leftStick = -RobotContainer.driveController.leftStick.getY(); 
        double rotation = RobotContainer.driveController.rightStick.getX();
        double forwardValue = leftStick * SpeedScale;
        double rotationValue = rotation * SpeedScale * 0.8;
        double leftValue = forwardValue + rotationValue;
        double rightValue = forwardValue - rotationValue;
        
        if (seen == 1.0)
            found = true;
        else
             topLeftMotor.set(ControlMode.PercentOutput, leftValue);
             bottomLeftMotor.set(ControlMode.PercentOutput, leftValue);
             topRightMotor.set(ControlMode.PercentOutput, -rightValue);
             bottomRightMotor.set(ControlMode.PercentOutput, -rightValue); 
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
