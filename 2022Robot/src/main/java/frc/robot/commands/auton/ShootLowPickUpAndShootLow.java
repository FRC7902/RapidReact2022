package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootLowWithWindUp;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ShootLowPickUpAndShootLow extends SequentialCommandGroup {
    public ShootLowPickUpAndShootLow(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new ShootLowAndPickUp(driveSubsystem, intakeSubsystem, transferSubsystem, shooterSubsystem),
            new TimedDriveWithSpeed(-0.5, 1, driveSubsystem),
            new ShootLowWithWindUp(transferSubsystem, shooterSubsystem)
        );
    }
}
