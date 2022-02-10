package frc.robot.subsystems.Auto;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.RobotMap;
import frc.robot.subsystems.ISubsystem;

public class AutoGo extends SubsystemBase implements ISubsystem{
    
    private TalonSRX topLeftMotor;
    private TalonSRX topRightMotor;
    private TalonSRX bottomLeftMotor;
    private TalonSRX bottomRightMotor;

    public AutoGo(){
        resetSubsystem();

        topLeftMotor = new TalonSRX(RobotMap.DRIVE_TRAIN_TOP_LEFT);
        topRightMotor = new TalonSRX(RobotMap.DRIVE_TRAIN_TOP_RIGHT);
        bottomLeftMotor = new TalonSRX(RobotMap.DRIVE_TRAIN_BOTTOM_LEFT);
        bottomRightMotor = new TalonSRX(RobotMap.DRIVE_TRAIN_BOTTOM_RIGHT);

    }
    
    @Override
    public void outputSmartdashboard() {
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
