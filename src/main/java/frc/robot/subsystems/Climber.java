package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase implements ISubsystem {

  private static int Step;

  private VictorSPX leftWinch;
  private VictorSPX rightWinch;
  private TalonSRX yellowArm;
  private TalonSRX redArm;

  public Climber() {
    leftWinch = new VictorSPX(RobotMap.CLIMBER_WINCH_LEFT);
    rightWinch = new VictorSPX(RobotMap.CLIMBER_WINCH_RIGHT);
    yellowArm = new TalonSRX(RobotMap.CLIMBER_ARM_RIGHT);
    redArm = new TalonSRX(RobotMap.CLIMBER_ARM_LEFT);
    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);
    yellowArm.setNeutralMode(NeutralMode.Brake);
    redArm.setNeutralMode(NeutralMode.Brake);
    Step = 0;
  }

  public void redArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    redArm.set(ControlMode.PercentOutput, speed);
  }

  public void yellowArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    yellowArm.set(ControlMode.PercentOutput, -speed);
  }

  public void redWinchRun(double speed){
    rightWinch.set(ControlMode.PercentOutput, speed);
  }
  public void yellowWinchRun(double speed){
    leftWinch.set(ControlMode.PercentOutput, speed);
  }

  public void stopClimbers(double speed) {
    speed = 0;
    rightWinch.set(ControlMode.PercentOutput, speed);
    leftWinch.set(ControlMode.PercentOutput, speed);
  }

  public void upStep() {
    Step++;
  }
  public void downStep() {
    Step--;
  }
  public int getStep(){
    return Step;
  }
  

  public void  runStepper(){
    if(Step==1){
      this.yellowWinchRun(-Constants.WINCH_SPEED);
    }else if(Step==2){
      this.yellowArm(-1);
    }
    else if(Step==3){
      this.yellowArm(-1);
      this.yellowWinchRun(Constants.WINCH_SPEED);
    }
    else if(Step==4){
      this.redWinchRun(-Constants.WINCH_SPEED);
    }
    else if(Step==5){
      this.redArm(1);
      new WaitCommand(2);
      this.redWinchRun(Constants.WINCH_SPEED);
    }
    else if(Step==6){
      this.redArm(1);
      this.yellowArm(-1);
      this.yellowWinchRun(-Constants.WINCH_SPEED);
      new WaitCommand(1);
      this.redWinchRun(Constants.WINCH_SPEED);
    }
    else if(Step==7){
      this.yellowWinchRun(-Constants.WINCH_SPEED);
    }
    else if(Step==8){  //Works until here
      this.yellowArm(1);
      this.yellowWinchRun(Constants.WINCH_SPEED);
    }
    else if(Step==9){
      this.yellowWinchRun(-Constants.WINCH_SPEED);
    }
    else if(Step==10){
      this.redArm(-1);
      this.redWinchRun(Constants.WINCH_SPEED);
    }
    else if(Step==11){
      this.redArm(1);
      this.redWinchRun(-Constants.WINCH_SPEED);
    }
    else if(Step==12){
      this.yellowWinchRun(Constants.WINCH_SPEED);
      this.redWinchRun(-Constants.WINCH_SPEED/2);
    }
    else if(Step==13){
      this.redWinchRun(-Constants.WINCH_SPEED);
    }
    else{
      Step=0;
    }
  }

  public void resetStep(){
    Step=0;
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
