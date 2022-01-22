// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
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

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private static Joystick driverStick = new Joystick(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    m_robotDrive.setDefaultCommand(

      new RunCommand(

        () -> m_robotDrive.drive(driverStick.getRawAxis(1), driverStick.getRawAxis((RobotBase.isReal()) ? 4 : 0))  
        , m_robotDrive)
        
      //   () -> m_robotDrive.drive(-m_stick.getX(), m_stick.getY())
      // , m_robotDrive)

    
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new JoystickButton(driverStick, Constants.RB)
    //   .whileHeld(m_shoot);

    // new JoystickButton(driverStick, Constants.LB)
    //   .whileHeld(m_intake);

    // new JoystickButton(driverStick, 1)
    //   .whileHeld(() -> m_robotArm.setArmAnglePIDF(30))
    //   .whenReleased(()-> m_robotArm.setMotor(0));

    new JoystickButton(driverStick, 2)
      .whenPressed(() -> m_robotDrive.resetOdometry(new Pose2d(5, 5, new Rotation2d())));


    // new JoystickButton(driverStick, 3)
    //   .whenPressed(() -> m_robotArm.setMotor(1))
    //   .whenReleased(() -> m_robotArm.setMotor(0));

    // new JoystickButton(driverStick, 4)
    //   .whenPressed(() -> m_robotArm.setMotor(-1))
    //   .whenReleased(() -> m_robotArm.setMotor(0));

    // new JoystickButton(m_stick, 5)
    //   .whenPressed(() -> m_robotElevator.raise(1.0))
    //   .whenReleased(() -> m_robotElevator.lower());
  }

  public DriveSubsystem getRobotDrive(){
    return m_robotDrive;
  }

  public Joystick getDriverJoystick(){
    return driverStick;
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
