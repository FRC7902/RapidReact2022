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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveAndTurn;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.ExampleCommand;

import frc.robot.commands.ExtendElevator;
import frc.robot.commands.RetractElevator;
import frc.robot.commands.TraverseRungs;
import frc.robot.subsystems.ClimbSubsystem;

import frc.robot.subsystems.CameraSubsystem;

import frc.robot.commands.LeaveTarmac;
import frc.robot.commands.PickUpAndShoot;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.SetElevatorToHeight;
import frc.robot.commands.SetElevatorToHeightPID;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAndLeave;
import frc.robot.commands.Spit;
import frc.robot.commands.Suck;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  
 

  //Initialize subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final TransferSubsystem m_robotTransfer = new TransferSubsystem();
  private final ShooterSubsystem m_robotShooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem(); 
  // private final CameraSubsystem m_robotCamera = new CameraSubsystem(); 
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  


  //Initialize commands
  private final DoNothing m_DoNothingAuto = new DoNothing();
  private final LeaveTarmac m_leaveTarmacAuto = new LeaveTarmac(m_robotDrive);
  private final ShootAndLeave m_shootAndLeaveAuto = new ShootAndLeave(m_robotTransfer, m_robotShooter, m_robotDrive);
  private final PickUpAndShoot m_pickUpAndShootAuto = new PickUpAndShoot(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  SendableChooser<Command> m_chooser = new SendableChooser<>();


  private final Joystick m_driverStick = new Joystick(Constants.IOConstants.kDriverStick);
  // private final XboxController m_climbController = new XboxController(Constants.IOConstants.kClimbStick);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.driveJoystick(m_driverStick.getRawAxis(1), m_driverStick.getRawAxis(4)), 
        m_robotDrive)
    );

    // m_climbSubsystem.setDefaultCommand(
    //   new RunCommand(
    //     () -> m_climbSubsystem.setWinches(
    //       m_climbController.getRawAxis(Constants.IOConstants.kRY), m_climbController.getRawAxis(Constants.IOConstants.kLY)), 
    //     m_climbSubsystem)
    // );



    m_chooser.setDefaultOption("Leave Tarmac", m_leaveTarmacAuto);
    m_chooser.addOption("Do nothing", m_DoNothingAuto);
    m_chooser.addOption("Just Shoot", new Shoot(m_robotTransfer, m_robotShooter).withTimeout(3.0));
    m_chooser.addOption("Shoot and Leave Tarmac", m_shootAndLeaveAuto);
    m_chooser.addOption("Pick Up and Shoot", m_pickUpAndShootAuto);


    Shuffleboard.getTab("CompetitionView").add(m_chooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    // Climb Elevator

    // new JoystickButton(m_climbController, Constants.IOConstants.kX)
    //   .whenPressed(() -> m_climbSubsystem.resetEncoder());

    
    // new JoystickButton(m_climbController, Constants.IOConstants.kRB) //Extend Elevator
    //   .whenHeld(new ExtendElevator(m_climbSubsystem));
    // new JoystickButton(m_climbController, Constants.IOConstants.kLB) //Retract Elevator
    //   .whenHeld(new RetractElevator(m_climbSubsystem));




    // Driver Stick
    new JoystickButton(m_driverStick, Constants.IOConstants.kA) //Retract intake
      .whenHeld(new RetractIntake(m_robotIntake));



    new JoystickButton(m_driverStick, Constants.IOConstants.kB) //Shoot
      .whenHeld(new Shoot(m_robotTransfer, m_robotShooter));


    new JoystickButton(m_driverStick, Constants.IOConstants.kX) //Toggle where the front of the robot is
      .whenPressed(new InstantCommand(() -> m_robotDrive.toggleRobotFront()));

    new JoystickButton(m_driverStick, Constants.IOConstants.kY) //Deploy intake
      .whenHeld(new DeployIntake(m_robotIntake));

    new JoystickButton(m_driverStick, Constants.IOConstants.kLB) //Spit
      .whenHeld(new Spit(m_robotIntake, m_robotTransfer));

    new JoystickButton(m_driverStick, Constants.IOConstants.kRB) //Suck
      .whenHeld(new Suck(m_robotIntake, m_robotTransfer));

    new JoystickButton(m_driverStick, Constants.IOConstants.kMENU);


    new JoystickButton(m_driverStick, Constants.IOConstants.kSTART);

    
    new JoystickButton(m_driverStick, Constants.IOConstants.kLA) //Activate/Deactivate Slow Drive
      .whenPressed(() -> m_robotDrive.activateSlowForward())
      .whenReleased(() -> m_robotDrive.deactivateSlowForward());

    new JoystickButton(m_driverStick, Constants.IOConstants.kRA) //Activate/Deactivate Slow Turn
      .whenPressed(() -> m_robotDrive.activateSlowTurn())
      .whenReleased(() -> m_robotDrive.deactivateSlowTurn());

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
