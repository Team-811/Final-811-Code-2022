package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.Shooter.GetDistance;

public class Shooter extends SubsystemBase implements ISubsystem {
    
    private WPI_TalonSRX LeftMotor;
    private WPI_TalonSRX RightMotor;

    public Shooter(){
       LeftMotor =  new WPI_TalonSRX(RobotMap.SHOOTER_MOTOR_LEFT);
       RightMotor = new WPI_TalonSRX(RobotMap.SHOOTER_MOTOR_RIGHT);
    }

    public void shooterSpin(double speed){
        LeftMotor.set(speed);
        RightMotor.set(-speed);
    }
    public void shooterStop(){
        LeftMotor.set(0);
        RightMotor.set(0);
    }


    @Override
    public void outputSmartdashboard() {
        double feet = GetDistance.Distance();
        SmartDashboard.putNumber("Distance", feet);
     //   SmartDashboard.putNumber("Shooter Speed", LeftMotor.getMotorOutputPercent());
       // SmartDashboard.putNumber("Shooter Temperature", LeftMotor.getMotorOutputPercent());
    }

    @Override
    public void zeroSensors() {
        shooterStop();
        
        
    }

    @Override
    public void resetSubsystem() {
        
    }

    @Override
    public void testSubsystem() {

        
    }
    
}