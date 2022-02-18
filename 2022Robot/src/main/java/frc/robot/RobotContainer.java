// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LeaveTarmac;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


  
  private final DriveToDistance m_driveForward = new DriveToDistance(2.0, m_robotDrive);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  private final LeaveTarmac m_leaveTarmac = new LeaveTarmac(m_robotDrive);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Joystick m_driverStick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.driveJoystick(-m_driverStick.getRawAxis(1), m_driverStick.getRawAxis(0)), 
        m_robotDrive)
    );


    m_chooser.setDefaultOption("Drive Forward 2 metres", m_driveForward);
    m_chooser.setDefaultOption("Leave Tarmac", m_leaveTarmac);

    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverStick, 1)
      .whenReleased(() -> m_robotDrive.zeroHeading(), m_robotDrive);

    new JoystickButton(m_driverStick, 2)
      .whenPressed(() -> m_robotDrive.resetOdometry(new Pose2d(7, 5, new Rotation2d(Units.degreesToRadians(-120)))));

    new JoystickButton(m_driverStick, 3)
      .whenPressed(() -> m_robotDrive.activateSlowForward(), m_robotDrive)
      .whenReleased(() -> m_robotDrive.deactivateSlowForward(), m_robotDrive);

    new JoystickButton(m_driverStick, 4)
      .whenPressed(() -> m_robotDrive.activateSlowTurn(), m_robotDrive)
      .whenReleased(() -> m_robotDrive.deactivateSlowTurn(), m_robotDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
