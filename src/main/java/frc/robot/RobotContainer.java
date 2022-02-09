// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Drivetrain.DriveLeft;
import frc.robot.commands.Drivetrain.DriveRight;
import frc.robot.commands.Drivetrain.DriveStop;
import frc.robot.commands.Drivetrain.DrivingCommand;
import frc.robot.commands.VisionTargeting.Cargo.Cat;
import frc.robot.commands.VisionTargeting.Hub.LimelightAim;
// import frc.robot.commands.LimelightAiming.LimelightAimX;
// import frc.robot.commands.LimelightAiming.LimelightAimY;
// import frc.robot.commands.LimelightAiming.LimelightSearch;
import frc.robot.controllers.BobXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private static final AHRS gyro = new AHRS();
  private static final Drivetrain drivetrain = new Drivetrain(gyro);
  // private static final Kicker kicker = new Kicker();
  // Compressor pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public static BobXboxController driveController;
  public static BobXboxController operatorController;

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(new DrivingCommand(drivetrain, driveController));
    configureButtonBindings();
    
    // pcmCompressor.enableDigital();
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

    //Button Map here
    // LimelightSearch limeSearch = new LimelightSearch(drivetrain);
    // LimelightAimY limeAimY = new LimelightAimY(drivetrain);
    // LimelightAimX limeAimX = new LimelightAimX(drivetrain);
    // Command[] LimelightAimingCommands = {limeSearch, limeAimY, limeAimX};
     driveController.bButton.whileHeld(new LimelightAim(drivetrain));
     driveController.xButton.whileHeld(new Cat(drivetrain));
     driveController.leftTriggerButton.whileHeld(new DriveLeft(drivetrain));
     driveController.leftTriggerButton.whenReleased(new DriveStop(drivetrain));
     driveController.rightTriggerButton.whileHeld(new DriveRight(drivetrain));
     driveController.rightTriggerButton.whenReleased(new DriveStop(drivetrain));
  //  driveController.bButton.whenPressed(new LimelightAimX2(drivetrain));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  public static void updateSmartdashboard() {
    drivetrain.outputSmartdashboard();
  }
}
