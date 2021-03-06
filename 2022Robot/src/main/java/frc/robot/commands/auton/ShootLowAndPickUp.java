package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.ShootLowWithWindUp;
import frc.robot.commands.Suck;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ShootLowAndPickUp extends SequentialCommandGroup {
    public ShootLowAndPickUp(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new ShootLowWithWindUp(transferSubsystem, shooterSubsystem).withTimeout(AutonConstants.shootTime),
            new DeployIntake(intakeSubsystem).withTimeout(AutonConstants.intakeDeployTime),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new TimedDriveWithSpeed(0.5, 1, driveSubsystem),
                    new WaitCommand(0.25)
                ),
                new Suck(intakeSubsystem, transferSubsystem)
            )
        );
    }
}
