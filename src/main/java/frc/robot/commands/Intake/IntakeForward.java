package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeForward extends CommandBase {
   private Intake requiredSubsystem;

   public IntakeForward(Intake m_SubsystemBase) {
       requiredSubsystem = m_SubsystemBase;
       addRequirements(requiredSubsystem);
   }
 
   @Override
   public void initialize() {}
 
   @Override
   public void execute() {
    requiredSubsystem.intakeSpin(Constants.REAL_INTAKE_SPEED);
    // if(requiredSubsystem.getLimitSwitch()==false){//might be false
    //   requiredSubsystem.backStop();
    // }else{
      requiredSubsystem.backSpin(Constants.INTAKE_SPEED);
    // }

}
 
   @Override
   public void end(boolean interrupted) {}
 
   @Override
   public boolean isFinished() {
     return true;
   }
 }