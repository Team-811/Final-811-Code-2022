package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.RobotMap;
import frc.robot.Vision.Ultrasonic;
public class AutoUltra extends SubsystemBase implements ISubsystem {
    
    private TalonSRX topLeftMotor;
    private TalonSRX topRightMotor;
    private TalonSRX bottomLeftMotor;
    private TalonSRX bottomRightMotor;

    private Ultrasonic ultrasonic;

    public AutoUltra(Ultrasonic m_ultrasonic){
        ultrasonic = m_ultrasonic;
        resetSubsystem();
        topLeftMotor = new TalonSRX(RobotMap.DRIVE_TRAIN_TOP_LEFT );
        topRightMotor= new TalonSRX(RobotMap.DRIVE_TRAIN_TOP_RIGHT );
        bottomLeftMotor = new TalonSRX(RobotMap.DRIVE_TRAIN_BOTTOM_LEFT );
        bottomRightMotor= new TalonSRX(RobotMap.DRIVE_TRAIN_BOTTOM_RIGHT );
        topLeftMotor.set(ControlMode.PercentOutput, 0.0f);
        topRightMotor .set(ControlMode.PercentOutput, 0.0f);
        bottomLeftMotor.set(ControlMode.PercentOutput, 0.0f);
        bottomRightMotor.set(ControlMode.PercentOutput, 0.0f);
    }


    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putNumber("Front Left Wheel", topLeftMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Front Right Wheel", -topRightMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Back Left Wheel", bottomLeftMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Back Right Wheel", -bottomRightMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Ultrasonic", ultrasonic.getDistance());
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void resetSubsystem() {
        
    }

    @Override
    public void testSubsystem() {

    }

}
