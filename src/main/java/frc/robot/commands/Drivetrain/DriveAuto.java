package frc.robot.commands.Drivetrain;


import edu.wpi.first.wpilibj2.command.CommandBase;

import java.net.SocketTimeoutException;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;


import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vision.LimelightFetch;

public class DriveAuto extends CommandBase{

    private Drivetrain requiredSubsystem;    
    double time, power, angle;
    double Kp = 0.025; //stc
    private static AHRS gyro = new AHRS();
    double targetAngle;

      
  public DriveAuto(Drivetrain m_SubsystemBase, double Itime, double Ipower) {
    requiredSubsystem = m_SubsystemBase;
    addRequirements(requiredSubsystem);

    power = Ipower;
  }

  @Override
  public void initialize() {
      gyro.reset();

  }

  @Override
  public void execute() {

    angle = gyro.getAngle();

   // RobotContainer.drivetrain.driveController(power, -angle * Kp);



       requiredSubsystem.driveWithMisery(-RobotContainer.driveController.leftStick.getY(),
       RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX());
       
      }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
