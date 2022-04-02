package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.ShootSetSpeed;
import frc.robot.commands.transfer.TransferUp;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ShootSetSpeedWithWindUp extends ParallelCommandGroup {
  public ShootSetSpeedWithWindUp(TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, int mainShooterSpeed) {
    addCommands(
      new ShootSetSpeed(shooterSubsystem, mainShooterSpeed),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new TransferUp(transferSubsystem)
      )
    );
  }
}
