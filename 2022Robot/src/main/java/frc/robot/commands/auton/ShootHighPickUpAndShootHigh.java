// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PullBack;
import frc.robot.commands.ShootHighWithWindUp;
import frc.robot.commands.Suck;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.shooter.ReverseShooter;
import frc.robot.commands.transfer.TransferDown;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHighPickUpAndShootHigh extends SequentialCommandGroup {
  /** Creates a new ShootHighPickUpAndShootHigh. */
  public ShootHighPickUpAndShootHigh(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootHighAndPickUp(driveSubsystem, intakeSubsystem, transferSubsystem, shooterSubsystem),
      new TimedDriveWithSpeed(-0.5, 1, driveSubsystem),
      new PullBack(transferSubsystem, shooterSubsystem).withTimeout(0.5),
      new ShootHighWithWindUp(transferSubsystem, shooterSubsystem).withTimeout(4)
    );
  }
}
