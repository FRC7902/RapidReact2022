// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants; 
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpAndShoot extends SequentialCommandGroup {
  /** Creates a new PickUpAndShoot. */
  public PickUpAndShoot(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeployIntake(intakeSubsystem).withTimeout(2),

      new ParallelRaceGroup(
        new TimedDriveWithSpeed(0.5, 3, driveSubsystem),
        new Suck(intakeSubsystem, transferSubsystem)
      ),

      new WaitCommand(1),

      new TimedDriveWithSpeed(-0.5, 3, driveSubsystem),

      new WaitCommand(1),

      new Shoot(transferSubsystem, shooterSubsystem, Constants.ShooterConstants.kLowSpeed).withTimeout(3.0)
      
    );
  }
}
