// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PullBack;
import frc.robot.commands.ShootHighWithWindUp;
import frc.robot.commands.Suck;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.auton.ShootHighPickUpAndShootHigh;
import frc.robot.commands.drivetrain.TimedTurnWithSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpAndShootHigh extends SequentialCommandGroup {
  /** Creates a new PickUpAndShootHigh. */
  public PickUpAndShootHigh(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    
    addCommands(


      //NEW THREE BALL HIGH

      new ShootHighPickUpAndShootHigh(driveSubsystem, intakeSubsystem, transferSubsystem, shooterSubsystem),

      new TimedTurnWithSpeed(0.5, 0.2, driveSubsystem),

      new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new TimedTurnWithSpeed(0.5, 0.2, driveSubsystem),
            new WaitCommand(0.1),
            new TimedDriveWithSpeed(0.5, 1.1, driveSubsystem),

            new WaitCommand(0.25)

          ),
          new Suck(intakeSubsystem, transferSubsystem)
      ),

      new TimedTurnWithSpeed(0.3, -0.15, driveSubsystem),

      new TimedDriveWithSpeed(0.5, 0.65, driveSubsystem)

    );
  }
}

// new DeployIntake(intakeSubsystem).withTimeout(AutonConstants.intakeDeployTime),
//       new ParallelDeadlineGroup(
//         new SequentialCommandGroup(
//           new TimedDriveWithSpeed(0.5, 0.5, driveSubsystem),
//           new TurnToAngle(-15, driveSubsystem),
//           new TimedDriveWithSpeed(0.5, 1, driveSubsystem),
//           new WaitCommand(0.25),
//           new TimedDriveWithSpeed(-0.5, 1, driveSubsystem),
//           new TurnToAngle(0, driveSubsystem),
//           new TimedDriveWithSpeed(-0.5, 0.5, driveSubsystem)
//         ), 
//         new Suck(intakeSubsystem, transferSubsystem)
//       ),
//       new PullBack(transferSubsystem, shooterSubsystem).withTimeout(0.5),
//       new ShootHighWithWindUp(transferSubsystem, shooterSubsystem).withTimeout(AutonConstants.shootTime)