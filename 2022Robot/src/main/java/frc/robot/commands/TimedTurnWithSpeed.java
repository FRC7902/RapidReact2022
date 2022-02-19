// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedTurnWithSpeed extends CommandBase {

  DriveSubsystem m_driveSubsystem;
  Timer timer = new Timer();
  double time;
  double speed;

  /** Creates a new TimedTurn. */
  public TimedTurnWithSpeed(double speed, double time, DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    this.time = time;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.stop();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.driveArcade(0, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
