package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DrivingCommand;
import frc.robot.commands.Auto.SimpleAuto;
import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.commands.Intake.Motors.IntakeForward;
import frc.robot.commands.Intake.Motors.IntakeReverse;
import frc.robot.commands.Intake.Motors.IntakeStop;
import frc.robot.commands.Intake.Pneumatics.IntakeToggle;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.controllers.BobXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final Drivetrain drivetrain = new Drivetrain(); 
  private static final Intake intake = new Intake();
  private static final Shooter shooter = new Shooter();
  private static final Climber climber = new Climber();

  Compressor pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM); // for testing compressor
  public static BobXboxController driveController;
  public static BobXboxController operatorController;
  Command m_autonomousCommand;
  SendableChooser<Command> m_Chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(new DrivingCommand(drivetrain));
    climber.setDefaultCommand(new ClimberCommand(climber));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveController = new BobXboxController(0, .3, .3);
    operatorController = new BobXboxController(1, .3, .3);
    //Button Mapping
    operatorController.xButton.whenPressed(new Shoot(shooter, intake));
    operatorController.yButton.whileHeld(new IntakeForward(intake));
    operatorController.yButton.whenReleased(new IntakeStop(intake));
    operatorController.rightTriggerButton.whenPressed(new IntakeToggle(intake));
    operatorController.bButton.whileHeld( new IntakeReverse(intake, shooter));
    operatorController.bButton.whenReleased(new IntakeStop(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_Chooser.setDefaultOption("Forward", new SimpleAuto(drivetrain));
    m_Chooser.addOption("Do Nothing :)", null);
  
    // An ExampleCommand will run in autonomous
    return new SimpleAuto(drivetrain);
    
  }
  public static void updateSmartdashboard() {
    drivetrain.outputSmartdashboard();
    shooter.outputSmartdashboard();
    intake.outputSmartdashboard();
  }
}
