package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PullBack;
import frc.robot.commands.ShootHighWithWindUp;
import frc.robot.commands.Suck;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourBallsTerminal extends SequentialCommandGroup{

    public FourBallsTerminal(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(

            new PickUpAndShootHigh(driveSubsystem, intakeSubsystem, transferSubsystem, shooterSubsystem),
            //re-uses this command, need to tune

            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new TimedDriveWithSpeed(0.5, 0.5, driveSubsystem),
                    new TurnToAngle(60, driveSubsystem),
                    new TimedDriveWithSpeed(0.5, 1, driveSubsystem),
                    new WaitCommand(0.25),
                    new TurnToAngle(75, driveSubsystem),
                    new TimedDriveWithSpeed(1, 1, driveSubsystem)
                ),
                new Suck(intakeSubsystem, transferSubsystem)
            ),
            new TimedDriveWithSpeed(1, 1.5, driveSubsystem),
            new TurnToAngle(0, driveSubsystem),
            new TimedDriveWithSpeed(0.5, 0.75, driveSubsystem),
            new PullBack(transferSubsystem, shooterSubsystem).withTimeout(0.5),
            new ShootHighWithWindUp(transferSubsystem, shooterSubsystem).withTimeout(4)
        );
    }



}