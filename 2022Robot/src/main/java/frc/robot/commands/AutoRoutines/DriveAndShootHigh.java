// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.TransferConstants;
import frc.robot.commands.Shoot;
import frc.robot.commands.TimedDriveWithSpeed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndShootHigh extends SequentialCommandGroup {
  /** Creates a new DriveAndShootHigh. */
  public DriveAndShootHigh(DriveSubsystem driveSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TimedDriveWithSpeed(0.5, 1, driveSubsystem),
      new RunCommand(() -> shooterSubsystem.shoot(Constants.ShooterConstants.kHighSpeed), shooterSubsystem).withTimeout(2),
      new Shoot(transferSubsystem, shooterSubsystem, Constants.ShooterConstants.kHighSpeed).withTimeout(6)
    );
  }
}
