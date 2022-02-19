// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class Shoot extends CommandBase {

  TransferSubsystem m_transferSubsystem;
  ShooterSubsystem m_shooterSubsystem;


  /** Creates a new Shoot. */
  public Shoot(TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
    m_transferSubsystem = transferSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transferSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transferSubsystem.stopTransfer();
    m_shooterSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transferSubsystem.transfer();
    m_shooterSubsystem.shoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transferSubsystem.stopTransfer();
    m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
