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


public class ThreeBallsSequential extends SequentialCommandGroup {
    public ThreeBallsSequential(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        
        double turnSpeed = 0.5;
        double turnAmt = 0.57;

        double driveSpeed = 0.5;
        double driveAmt = 1.90;

        addCommands(


            new ShootHighAndPickUp(driveSubsystem, intakeSubsystem, transferSubsystem, shooterSubsystem),
            
            //new TimedDriveWithSpeed(-0.5, 0.85, driveSubsystem),
            //new WaitCommand(0.25),
            //new ShootMaintained(Constants.ShooterConstants.kHighUnitsPerSec, shooterSubsystem, transferSubsystem).withTimeout(2.5),
            // ^ from two balls high

            new TimedTurnWithSpeed(turnSpeed,turnAmt, driveSubsystem),

            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new TimedDriveWithSpeed(driveSpeed, driveAmt, driveSubsystem),
                    new WaitCommand(1.1)
                ),
                new Suck(intakeSubsystem, transferSubsystem)
            ),
            new TimedDriveWithSpeed(-driveSpeed,(driveAmt),driveSubsystem),
            new WaitCommand(0.25),
            new TimedTurnWithSpeed(-turnSpeed,(turnAmt),driveSubsystem),

            //shoot 3rd ball
            new PullBack(transferSubsystem, shooterSubsystem).withTimeout(0.2),
            new ShootMaintained(Constants.ShooterConstants.kHighUnitsPerSec, shooterSubsystem, transferSubsystem).withTimeout(3)

            //new TimedDriveWithSpeed(0.7, 0.9, driveSubsystem) //clear tarmac at end
            
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