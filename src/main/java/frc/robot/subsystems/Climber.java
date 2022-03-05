package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase implements ISubsystem {

  private VictorSPX leftWinch;
  private VictorSPX rightWinch;
  private TalonSRX rightArm;
  private TalonSRX leftArm;

  public Climber() {
    leftWinch = new VictorSPX(RobotMap.CLIMBER_WINCH_LEFT);
    rightWinch = new VictorSPX(RobotMap.CLIMBER_WINCH_RIGHT);
    rightArm = new TalonSRX(RobotMap.CLIMBER_ARM_RIGHT);
    leftArm = new TalonSRX(RobotMap.CLIMBER_ARM_LEFT);
    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);
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
    rightWinch.set(ControlMode.PercentOutput, speed);
  }
  public void leftWinchRun(double speed){
    leftWinch.set(ControlMode.PercentOutput, speed);
  }

  public void stopClimbers(double speed) {
    speed = 0;
    rightWinch.set(ControlMode.PercentOutput, speed);
    leftWinch.set(ControlMode.PercentOutput, speed);
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
