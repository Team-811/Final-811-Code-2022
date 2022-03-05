package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.VisionProcessing.Distance;

public class Shooter extends SubsystemBase implements ISubsystem {
    
    private WPI_TalonSRX LeftMotor;
    private WPI_TalonSRX RightMotor;

    public Shooter(){
       LeftMotor =  new WPI_TalonSRX(RobotMap.SHOOTER_MOTOR_LEFT);
       RightMotor = new WPI_TalonSRX(RobotMap.SHOOTER_MOTOR_RIGHT);
    }

    public void shooterSpin(double speed){
        LeftMotor.set(-speed);
        RightMotor.set(speed);
    }

    public void shooterSpinShoot(double speed){
        LeftMotor.set(speed);
        RightMotor.set(-speed);
    }

    public void shooterStop(){
        LeftMotor.set(0);
        RightMotor.set(0);
    }

    @Override
    public void outputSmartdashboard() {
        double feet = Distance.get();
        SmartDashboard.putNumber("Distance", feet);
    }

    @Override
    public void zeroSensors() {
        shooterStop(); 
    }

    @Override
    public void resetSubsystem() {}

    @Override
    public void testSubsystem() {}
}