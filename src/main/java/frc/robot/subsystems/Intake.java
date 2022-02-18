package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase implements ISubsystem {

    private CANSparkMax intakeMotor;
    private CANSparkMax backIntakeMotor;
    private DigitalInput limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);
    private DoubleSolenoid extendPiston; 


    public Intake(){
       intakeMotor =  new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
       backIntakeMotor = new CANSparkMax(RobotMap.INTAKE_BACK_MOTOR, MotorType.kBrushless);

       extendPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_PISTON_EXTENTION, RobotMap.INTAKE_PISTON_RETRACTION);

    }
    public void extendIntake() {
        
        extendPiston.set(Value.kReverse);
    }
    public void retractIntake() {
        extendPiston.set(Value.kForward);
    }
    
    public void stopPistonIntake(){
        extendPiston.set(Value.kOff);
    }


    public void intakeSpin(double speed){
        intakeMotor.set(speed);
    }
    public void intakeStop(){
        intakeMotor.set(0);
    }
    public void backSpin(double speed){
        backIntakeMotor.set(speed);
    }
    public void backStop(){
        backIntakeMotor.set(0);
    }
    public boolean getLimitSwitch(){
        return limitSwitch.get();
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
