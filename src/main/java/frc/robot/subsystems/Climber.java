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
  private TalonSRX rightArm;
  private TalonSRX leftArm;

  public Climber() {
    leftWinch = new VictorSPX(RobotMap.CLIMBER_WINCH_LEFT);
    rightWinch = new VictorSPX(RobotMap.CLIMBER_WINCH_RIGHT);
    rightArm = new TalonSRX(RobotMap.CLIMBER_ARM_RIGHT);
    leftArm = new TalonSRX(RobotMap.CLIMBER_ARM_LEFT);
    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);
    rightArm.setNeutralMode(NeutralMode.Brake);
    leftArm.setNeutralMode(NeutralMode.Brake);
    Step = 0;
  }

  public void leftArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    leftArm.set(ControlMode.PercentOutput, speed);
  }

  public void rightArm(double joystick){
    double speed = joystick * Constants.ARM_SPEED_SCALE;
    rightArm.set(ControlMode.PercentOutput, -speed);
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

  public void upStep() {
    Step++;
  }

  public int getStep(){
    return Step;
  }

  public void  runStepper(){
    //Negative is left on diagram for arms, positive extends the winches
    if(Step==1){
      // this.leftArm(1);
    }else if(Step==2){
      // this.leftArm(1);
      this.leftWinchRun(-Constants.WINCH_SPEED);
    }else if(Step==3){
      // this.leftArm(1);
      this.rightArm(-1);
    }
    else if(Step==4){
      // this.leftArm(1);
      this.rightArm(-1);
      this.leftWinchRun(Constants.WINCH_SPEED);
      // this.rightWinchRun(Constants.WINCH_SPEED);
    }
    else if(Step==5){
      this.stopClimbers(0);
    }
    else if(Step==6){
      this.rightWinchRun(-Constants.WINCH_SPEED);
    }
    else if(Step==7){
      this.leftArm(1);
      new WaitCommand(2);
      this.rightWinchRun(Constants.WINCH_SPEED);
    }
    else if(Step==8){
      this.leftArm(1);
      this.rightArm(-1);
      this.leftWinchRun(-Constants.WINCH_SPEED);
      new WaitCommand(1);
      this.rightWinchRun(Constants.WINCH_SPEED);
    }
    // else if(Step==8){
    //   this.leftArm(-1);
    // }
    // else if(Step==9){
    //   this.leftWinchRun(Constants.WINCH_SPEED);
    // }
    // else if(Step==10){
    //   this.leftArm(-1);
    //   this.leftWinchRun(-Constants.WINCH_SPEED);
    // }else if(Step==11){
    //   this.leftArm(1);
    // }else if(Step==12){
    //   this.leftArm(1);
    //   this.leftWinchRun(Constants.WINCH_SPEED);
    //   this.rightArm(1);
    // }else if(Step==13){
    //   this.rightArm(1);
    //   this.rightWinchRun(-Constants.WINCH_SPEED);
    // }
    // else if(Step==14){
    //   this.stopClimbers(0);
    // }
    // else if(Step==15){
    //   this.rightArm(-1);
    // }
    // else if(Step==16){
    //   this.rightArm(-1);
    //   this.rightWinchRun(-Constants.WINCH_SPEED);
    // }
    // else if(Step==17){
    //   this.leftArm(-1);
    //   this.rightWinchRun(Constants.WINCH_SPEED);
    //   this.leftWinchRun(Constants.WINCH_SPEED);
    // }
    // else if(Step==18){
    //   this.leftArm(-1);
    // }
    // else if(Step==19){
    //   this.leftWinchRun(Constants.WINCH_SPEED);
    // }
    // else if(Step==20){
    //   this.leftArm(-1);
    //   this.leftWinchRun(-Constants.WINCH_SPEED);
    // }else if(Step==21){
    //   this.leftArm(1);
    // }else if(Step==22){
    //   this.leftArm(1);
    //   this.rightArm(-1);
    //   this.leftWinchRun(Constants.WINCH_SPEED);
    // }else if(Step==23){
    //   this.rightArm(-1);
    //   this.leftArm(1);
    //   this.rightWinchRun(-Constants.WINCH_SPEED);
    // }
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
