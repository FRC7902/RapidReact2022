// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class Suck extends CommandBase {

  IntakeSubsystem m_intakeSubsystem;
  TransferSubsystem m_transferSubsystem;

  /** Creates a new Suck. */
  public Suck(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_transferSubsystem = transferSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, transferSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.stopIntakePower();
    m_transferSubsystem.stopTransfer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.suck();
    m_transferSubsystem.transfer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntakePower();
    m_transferSubsystem.stopTransfer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}