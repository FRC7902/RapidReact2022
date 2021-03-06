// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ShootMaintained extends CommandBase {
  ShooterSubsystem m_shooterSubsystem;
  TransferSubsystem m_transferSubsystem;
  int m_speed;

  /** Creates a new ShootMaintained. */
  public ShootMaintained(int speed, ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_transferSubsystem = transferSubsystem;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_shooterSubsystem, m_transferSubsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.stop();
    m_shooterSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.bangSpeed(m_speed);
    System.out.println ("Executing");
    
    if(m_shooterSubsystem.atTargetSpeed()){
      m_transferSubsystem.setSpeed(1);
      System.out.println ("Reached target");
    }else{
      m_transferSubsystem.stop();
      System.out.println ("Not at target");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.brake();
    m_shooterSubsystem.stop();
    m_transferSubsystem.stop();
    System.out.println("Ended");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
