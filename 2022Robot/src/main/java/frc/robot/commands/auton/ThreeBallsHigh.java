package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.PullBack;
import frc.robot.commands.ShootHighWithWindUp;
import frc.robot.commands.Suck;
import frc.robot.commands.drivetrain.DriveToDistance;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.commands.drivetrain.TimedTurnWithSpeed;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.commands.ShootMaintained;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auton.ShootHighAndPickUp;


public class ThreeBallsHigh extends SequentialCommandGroup {
    public ThreeBallsHigh(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        
        addCommands(


            new ShootHighAndPickUp(driveSubsystem, intakeSubsystem, transferSubsystem, shooterSubsystem),
            
            new TimedTurnWithSpeed(0.5, 0.55, driveSubsystem),

            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new TimedDriveWithSpeed(0.5, 1.8, driveSubsystem),
                    new WaitCommand(0.5)
                ),
                new Suck(intakeSubsystem, transferSubsystem)
            ),

            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new TimedTurnWithSpeed(-0.5, 0.42, driveSubsystem),
                    new WaitCommand(0.1),
                    new TimedDriveWithSpeed(-0.5, 0.85, driveSubsystem)
                ),
                new Suck(intakeSubsystem, transferSubsystem)
            ),

            new PullBack(transferSubsystem, shooterSubsystem).withTimeout(0.2),
            new ShootMaintained(Constants.ShooterConstants.kHighUnitsPerSec, shooterSubsystem, transferSubsystem).withTimeout(4)

            //new DriveToDistance(1000, driveSubsystem)

            // new TurnToAngle(60, driveSubsystem),
            // new TimedDriveWithSpeed(0.5, 0.5, driveSubsystem),

            // new TurnToAngle(0, driveSubsystem),
            // new TimedDriveWithSpeed(-0.5, 0.5, driveSubsystem),
        );
    }
}




// new ShootHighWithWindUp(transferSubsystem, shooterSubsystem).withTimeout(AutonConstants.shootTime),
            // new ParallelDeadlineGroup(
            //     new SequentialCommandGroup(
            //         new TimedDriveWithSpeed(0.5, 1, driveSubsystem),
            //         new WaitCommand(0.25),
            //         new TurnToAngle(105, driveSubsystem),
            //         new TimedDriveWithSpeed(0.5, 1, driveSubsystem)
            //     ),
            //     new Suck(intakeSubsystem, transferSubsystem)
            // ),