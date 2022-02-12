package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

public class Intake extends SubsystemBase implements ISubsystem {

    private CANSparkMax intakeMotor;

    public Intake(){
    //    intakeMotor =  new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
    }

    public void intakeSpin(double speed){
        intakeMotor.set(speed);
    }
    public void intakeStop(){
        intakeMotor.set(0);
    }


    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putNumber("Intake Speed", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake Temperature", intakeMotor.getMotorTemperature());
    }

    @Override
    public void zeroSensors() {
        intakeStop();
        
        
    }

    @Override
    public void resetSubsystem() {
        
    }

    @Override
    public void testSubsystem() {

        
    }
    
}
