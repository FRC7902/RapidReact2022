package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.ShootMaintained;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants.AutonConstants;
import frc.robot.commands.ShootHighWithWindUp;
import frc.robot.commands.Suck;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.commands.drivetrain.TimedTurnWithSpeed;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;


public class ShootHighAndPickUp extends SequentialCommandGroup {
    public ShootHighAndPickUp(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            
            new ParallelCommandGroup(
                new ShootMaintained(Constants.ShooterConstants.kHighUnitsPerSec, shooterSubsystem, transferSubsystem).withTimeout(1.8),
                new DeployIntake(intakeSubsystem).withTimeout(AutonConstants.intakeDeployTime)
            ),
            
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new TimedDriveWithSpeed(0.5, 1.0, driveSubsystem),
                    new WaitCommand(0.55)
                ),
                new Suck(intakeSubsystem, transferSubsystem)
            ),
            new TimedDriveWithSpeed(-0.5, 0.85, driveSubsystem),
            new TimedTurnWithSpeed(-0.5, 0.09, driveSubsystem),
            //new PullBack(transferSubsystem, shooterSubsystem).withTimeout(0.5),
      
            new ShootMaintained(Constants.ShooterConstants.kHighUnitsPerSec, shooterSubsystem, transferSubsystem).withTimeout(2.5)
      
        );
    }
}
