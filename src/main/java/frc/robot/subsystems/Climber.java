package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.CANVenom;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase implements ISubsystem {

  private CANVenom leftWinch;
  private CANVenom rightWinch;
  private TalonSRX rightArm;
  private TalonSRX leftArm;

  public Climber() {
    leftWinch = new CANVenom(RobotMap.CLIMBER_WINCH_LEFT);
    rightWinch = new CANVenom(RobotMap.CLIMBER_WINCH_RIGHT);
    rightArm = new TalonSRX(RobotMap.CLIMBER_ARM_RIGHT);
    leftArm = new TalonSRX(RobotMap.CLIMBER_ARM_LEFT);
  }

  public void leftArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    leftArm.set(ControlMode.PercentOutput, speed);
  }

  public void rightArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    rightArm.set(ControlMode.PercentOutput, speed);
  }

  public void rightWinchRun(double speed){
    if(speed < 0){
      // if(!rightLimit.get()){
        rightWinch.set(speed);
      // }
    }else{
      rightWinch.set(speed);
    }
  }
  public void leftWinchRun(double speed){
      if(speed < 0){
        // if(!leftLimit.get()){
          leftWinch.set(speed);
        // }
      }else{
        leftWinch.set(speed);
  }
  }

  public void stopClimbers(double speed) {
    leftWinch.set(0);
    rightWinch.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void outputSmartdashboard() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void resetSubsystem() {}

  @Override
  public void testSubsystem() {}
}
