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
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ThreeBallsHigh extends SequentialCommandGroup {
    public ThreeBallsHigh(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new ShootHighWithWindUp(transferSubsystem, shooterSubsystem),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new TimedDriveWithSpeed(0.5, 1, driveSubsystem),
                    new WaitCommand(0.25),
                    new TurnToAngle(105, driveSubsystem),
                    new TimedDriveWithSpeed(0.5, 1, driveSubsystem)
                ),
                new Suck(intakeSubsystem, transferSubsystem)
            ),
            new TurnToAngle(60, driveSubsystem),
            new TimedDriveWithSpeed(-0.5, 0.5, driveSubsystem),
            new TurnToAngle(0, driveSubsystem),
            new TimedDriveWithSpeed(-0.5, 0.5, driveSubsystem),
            new PullBack(transferSubsystem, shooterSubsystem).withTimeout(0.5),
            new ShootHighWithWindUp(transferSubsystem, shooterSubsystem).withTimeout(4)
        );
    }
}
