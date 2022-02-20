// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendElevator;
import frc.robot.commands.RetractElevator;
import frc.robot.commands.TraverseRungs;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final ExtendElevator m_extendElevator = new ExtendElevator(m_climbSubsystem);
  private final RetractElevator m_retractElevator = new RetractElevator(m_climbSubsystem);
  private final TraverseRungs m_traverseRungs = new TraverseRungs(m_climbSubsystem);

  private final XboxController m_climbController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Elevator
    new JoystickButton(m_climbController, Constants.RB).whenPressed(m_extendElevator.withTimeout(3));
    new JoystickButton(m_climbController, Constants.LB).whenPressed(m_retractElevator.withTimeout(3));
    new JoystickButton(m_climbController, Constants.RT).whileHeld(m_extendElevator).whenReleased(() -> m_climbSubsystem.stopElevator());
    new JoystickButton(m_climbController, Constants.LT).whileHeld(m_retractElevator).whenReleased(() -> m_climbSubsystem.stopElevator());
    // Main Winch
    new JoystickButton(m_climbController, Constants.Y).whileHeld(() -> m_climbSubsystem.setMainWinch(1.0)).whenReleased(() -> m_climbSubsystem.stopMainWinch());
    new JoystickButton(m_climbController, Constants.A).whileHeld(() -> m_climbSubsystem.setMainWinch(-1.0)).whenReleased(() -> m_climbSubsystem.stopMainWinch());
    // Adjustment Winch
    new JoystickButton(m_climbController, Constants.B).whileHeld(() -> m_climbSubsystem.setAdjustmentWinch(1.0)).whenReleased(() -> m_climbSubsystem.stopAdjustmentWinch());
    new JoystickButton(m_climbController, Constants.X).whileHeld(() -> m_climbSubsystem.setAdjustmentWinch(-1.0)).whenReleased(() -> m_climbSubsystem.stopAdjustmentWinch());
    // Auto-Traverse Rungs
    new JoystickButton(m_climbController, Constants.S).whenPressed(m_traverseRungs);
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
}
