// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndLeave extends SequentialCommandGroup {
  /** Creates a new ShootAndLeave. */
  public ShootAndLeave(TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Shoot(transferSubsystem, shooterSubsystem, Constants.ShooterConstants.kLowSpeed).withTimeout(4),

      new TimedDriveWithSpeed(0.5, 1.5, driveSubsystem)
    );
  }
}
