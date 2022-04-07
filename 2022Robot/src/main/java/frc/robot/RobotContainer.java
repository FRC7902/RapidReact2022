// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auton.FourBallsHigh;
import frc.robot.commands.auton.PickUpAndShootHigh;
import frc.robot.commands.auton.ShootHighAndLeave;
import frc.robot.commands.auton.ShootHighAndPickUp;
import frc.robot.commands.auton.ShootHighPickUpAndShootHigh;
import frc.robot.commands.auton.ShootLowAndLeave;
import frc.robot.commands.auton.ShootLowAndPickUp;
import frc.robot.commands.auton.ShootLowPickUpAndShootLow;
import frc.robot.commands.drivetrain.DriveToDistance;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.commands.elevator.ExtendElevator;
import frc.robot.commands.elevator.RetractElevator;
import frc.robot.commands.elevator.SetElevatorToHeight;
import frc.robot.commands.elevator.SetElevatorToHeightPID;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootHigh;
import frc.robot.commands.transfer.TransferDown;
import frc.robot.commands.transfer.TransferUp;
import frc.robot.commands.winches.RollBackwards;
import frc.robot.commands.winches.RollForward;
import frc.robot.commands.winches.RunWinches;
import frc.robot.commands.winches.WinchIn;
import frc.robot.commands.winches.WinchOut;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WinchSubsystem;

import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.LowerElevatorAndWinchesInSync;
import frc.robot.commands.PullBack;
import frc.robot.commands.RaiseElevatorAndWinchesInSync;
import frc.robot.commands.ShootHighWithWindUp;
import frc.robot.commands.ShootLowWithWindUp;
import frc.robot.commands.ShootSetSpeedWithWindUp;
import frc.robot.commands.Spit;
import frc.robot.commands.Suck;
import frc.robot.commands.TraverseRungs;
import frc.robot.commands.autoclimb.AutoHighAllStages;
import frc.robot.commands.autoclimb.AutoHighStage1;
import frc.robot.commands.autoclimb.AutoHighStage2;
import frc.robot.commands.autoclimb.AutoHighStage3;
import frc.robot.commands.autoclimb.AutoHighStage4;
import frc.robot.commands.autoclimb.AutoHighStage5;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem(); 
  private final WinchSubsystem m_robotWinch = new WinchSubsystem();
  private final CameraSubsystem m_robotCamera = new CameraSubsystem(); 
  


  //Initialize commands
  private final ShootLowAndLeave m_shootLowAndLeaveAuto = new ShootLowAndLeave(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);
  private final ShootLowAndPickUp m_shootLowAndPickUpAuto = new ShootLowAndPickUp(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);
  private final ShootLowPickUpAndShootLow m_shootLowPickUpAndShootLowAuto = new ShootLowPickUpAndShootLow(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);
  private final ShootHighAndLeave m_shootHighAndLeaveAuto = new ShootHighAndLeave(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);
  private final ShootHighAndPickUp m_shootHighAndPickUpAuto = new ShootHighAndPickUp(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);
  private final ShootHighPickUpAndShootHigh m_shootHighPickUpAndShootHighAuto = new ShootHighPickUpAndShootHigh(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);
  private final PickUpAndShootHigh m_pickUpAndShootHighAuto = new PickUpAndShootHigh(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);
  private final FourBallsHigh m_fourBallsHighAuto = new FourBallsHigh(m_robotDrive, m_robotIntake, m_robotTransfer, m_robotShooter);


  SendableChooser<Command> m_chooser = new SendableChooser<>();


  private final Joystick m_driverStick = new Joystick(Constants.IOConstants.kDriverStick);
  private final XboxController m_climberStick = new XboxController(Constants.IOConstants.kClimbStick);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.driveJoystick(m_driverStick.getRawAxis(Constants.IOConstants.kLY), m_driverStick.getRawAxis(Constants.IOConstants.kRX)), 
        m_robotDrive)
    );


    m_robotWinch.setDefaultCommand(
      // new RunWinches(m_climberStick.getRawAxis(Constants.IOConstants.kRY), m_climberStick.getRawAxis(Constants.IOConstants.kLY), m_robotWinch)
      new RunCommand(() -> m_robotWinch.setWinches(m_climberStick.getRawAxis(Constants.IOConstants.kLY)*0.9, m_climberStick.getRawAxis(Constants.IOConstants.kRY)), m_robotWinch)
    );



    m_chooser.setDefaultOption("Leave Tarmac", new TimedDriveWithSpeed(0.5, 1.5, m_robotDrive));
    m_chooser.addOption("Do Nothing", new WaitCommand(1));
    m_chooser.addOption("Shoot Low", new ShootLowWithWindUp(m_robotTransfer, m_robotShooter).withTimeout(4));
    m_chooser.addOption("Shoot High", new ShootHighWithWindUp(m_robotTransfer, m_robotShooter).withTimeout(4));
    m_chooser.addOption("Shoot Low and Leave Tarmac", m_shootLowAndLeaveAuto);
    m_chooser.addOption("Shoot High and Leave Tarmac", m_shootHighAndLeaveAuto);
    m_chooser.addOption("Shoot Low and Pick Up", m_shootLowAndPickUpAuto);
    m_chooser.addOption("Shoot High and Pick Up", m_shootHighAndPickUpAuto);
    m_chooser.addOption("Shoot Low, Pick Up, and Shoot Low", m_shootLowPickUpAndShootLowAuto);
    m_chooser.addOption("Shoot High, Pick Up, and Shoot High", m_shootHighPickUpAndShootHighAuto);
    m_chooser.addOption("Pick Up And Shoot High", m_pickUpAndShootHighAuto);
    m_chooser.addOption("Four Balls High", m_fourBallsHighAuto);


    Shuffleboard.getTab("CompetitionView").add(m_chooser);

    SmartDashboard.putData("ClimbSubsystem/AutoHigh Stage 1", new AutoHighStage1(m_robotElevator, m_robotWinch));
    SmartDashboard.putData("ClimbSubsystem/AutoHigh Stage 2", new AutoHighStage2(m_robotDrive, m_robotElevator, m_robotWinch));
    SmartDashboard.putData("ClimbSubsystem/AutoHigh Stage 3", new AutoHighStage3(m_robotElevator, m_robotWinch));
    SmartDashboard.putData("ClimbSubsystem/AutoHigh Stage 4", new AutoHighStage4(m_robotElevator));
    SmartDashboard.putData("ClimbSubsystem/AutoHigh Stage 5", new AutoHighStage5(m_robotWinch));
    SmartDashboard.putData("ClimbSubsystem/AutoHigh All Stages", new AutoHighAllStages(m_robotElevator, m_robotWinch, m_robotDrive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    //CLIMBER STICK
    new JoystickButton(m_climberStick, Constants.IOConstants.kX)
      .whenPressed(new AutoHighStage1(m_robotElevator, m_robotWinch));

    new JoystickButton(m_climberStick, Constants.IOConstants.kY) //Winch Up 
      .whenPressed(new AutoHighAllStages(m_robotElevator, m_robotWinch, m_robotDrive));

    new JoystickButton(m_climberStick, Constants.IOConstants.kA) //Winch Down 
      .whenHeld(new TransferUp(m_robotTransfer));

    new JoystickButton(m_climberStick, Constants.IOConstants.kB)
      .whenHeld(new TransferDown(m_robotTransfer));
    
    new JoystickButton(m_climberStick, Constants.IOConstants.kRB) //Extend Elevator
      .whenHeld(new ExtendElevator(m_robotElevator));
    new JoystickButton(m_climberStick, Constants.IOConstants.kLB) //Retract Elevator
      .whenHeld(new RetractElevator(m_robotElevator));

    new JoystickButton(m_climberStick, Constants.IOConstants.kSTART) //Reset Encoder
      .whenPressed(() -> m_robotElevator.resetEncoder());

    new Trigger(() -> m_climberStick.getRawAxis(Constants.IOConstants.kRT) > 0.01) // Extend Elevator and Winches in sync
      .whileActiveOnce(new RaiseElevatorAndWinchesInSync(m_robotElevator, m_robotWinch));
      
    new Trigger(() -> m_climberStick.getRawAxis(Constants.IOConstants.kLT) > 0.01) //Retract Elevator and Winches in sync
      .whileActiveOnce(new LowerElevatorAndWinchesInSync(m_robotElevator, m_robotWinch));

    // new Trigger(() -> m_climberStick.getRawAxis(Constants.IOConstants.kDY) > 0)
    //   .whileActiveOnce(new RollForward(m_robotWinch));
    
    // new Trigger(() -> m_climberStick.getRawAxis(Constants.IOConstants.kDY) < 0)
    //   .whileActiveOnce(new RollBackwards(m_robotWinch));


    //DRIVER STICK
    new JoystickButton(m_driverStick, Constants.IOConstants.kA) //Retract intake
      .whenHeld(new RetractIntake(m_robotIntake));
    
    new JoystickButton(m_driverStick, Constants.IOConstants.kB) //Shoot High
      .whenHeld(new ShootLowWithWindUp(m_robotTransfer, m_robotShooter));

    new JoystickButton(m_driverStick, Constants.IOConstants.kX) // Reverse shooter and transfer
      .whenHeld(new PullBack(m_robotTransfer, m_robotShooter));

    new JoystickButton(m_driverStick, Constants.IOConstants.kY) //Deploy intake
      .whenHeld(new DeployIntake(m_robotIntake));

    new JoystickButton(m_driverStick, Constants.IOConstants.kLB) //Spit
      .whenHeld(new Spit(m_robotIntake, m_robotTransfer));

    new JoystickButton(m_driverStick, Constants.IOConstants.kRB) //Suck
      .whenHeld(new Suck(m_robotIntake, m_robotTransfer));

    new JoystickButton(m_driverStick, Constants.IOConstants.kMENU) //Toggle where the front of the robot is
      .whenPressed(new InstantCommand(() -> m_robotDrive.toggleRobotFront()));

    new JoystickButton(m_driverStick, Constants.IOConstants.kSTART) //Shoot Low
      .whenHeld(new ShootHighWithWindUp(m_robotTransfer, m_robotShooter));

    new JoystickButton(m_driverStick, Constants.IOConstants.kLA) //Activate/Deactivate Slow Drive
      .whenPressed(() -> m_robotDrive.activateSlowForward())
      .whenReleased(() -> m_robotDrive.deactivateSlowForward());

    new JoystickButton(m_driverStick, Constants.IOConstants.kRA) //Activate/Deactivate Slow Turn
      .whenPressed(() -> m_robotDrive.activateSlowTurn())
      .whenReleased(() -> m_robotDrive.deactivateSlowTurn());


    new Trigger(() -> m_driverStick.getRawAxis(Constants.IOConstants.kRT) > 0.01)
      .whileActiveOnce((new ShootSetSpeedWithWindUp(m_robotTransfer, m_robotShooter, 4000)));

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
