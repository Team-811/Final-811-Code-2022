package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.VisionProcessing.Limelight;
import frc.robot.VisionProcessing.Lemonlight;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  @SuppressWarnings("unused")
  private RobotContainer m_robotContainer= new RobotContainer();
  // private static final Drivetrain drivetrain = new Drivetrain();
  // private static final Intake intake = new Intake();
  // private static final Shooter shooter = new Shooter();
  
  Command m_autonomousCommand;
  SendableChooser<Command> m_Chooser = new SendableChooser<>();
 // public static Drivetrain drivetrain;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


  
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    // m_Chooser.setDefaultOption("Forward", new SimpleAuto(drivetrain));
    // m_Chooser.addOption("Backward", new BackwardsAuto(drivetrain));
    // m_Chooser.addOption("Complex", new ComplexAuto(drivetrain, intake, shooter));
    m_Chooser.addOption("Do Nothing :(", null);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //drivetrain = Drivetrain.getInstance();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // Limelight.Morse();
    CommandScheduler.getInstance().run();
    RobotContainer.updateSmartdashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Limelight.Off();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Lemonlight.setTeam();
    Limelight.On(); 
    SmartDashboard.putData("Auto mode", m_Chooser);
    m_autonomousCommand = m_Chooser.getSelected();

    if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    

  }

  @Override
  public void teleopInit() {
    Limelight.On();
    Lemonlight.setTeam();
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


 
}
