// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TransferSubsystem;

public class TransferDown extends CommandBase {
  TransferSubsystem m_transferSubsystem;

  /** Creates a new TransferDown. */
  public TransferDown(TransferSubsystem transferSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_transferSubsystem = transferSubsystem;

    addRequirements(transferSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transferSubsystem.stop();

    System.out.println("TransferSubsystem: Started transferring down");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transferSubsystem.setSpeed(Constants.TransferConstants.kVertBackwardsSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transferSubsystem.stop();

    System.out.println("TransferSubsystem: Finished transferring down");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
