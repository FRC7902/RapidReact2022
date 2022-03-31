// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootLow extends CommandBase {
  ShooterSubsystem m_shooterSubsystem;

  /** Creates a new ShootLow. */
  public ShootLow(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.stop();

    System.out.println("ShooterSubsystem: Started shooting low");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setSpeed(Constants.ShooterConstants.kLowSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();

    System.out.println("ShooterSubsystem: Finished shooting low");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
