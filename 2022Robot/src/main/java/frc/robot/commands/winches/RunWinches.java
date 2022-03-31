// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.winches;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class RunWinches extends CommandBase {
  WinchSubsystem m_winchSubsystem;
  double mainSpeed, adjSpeed;

  /** Creates a new WinchUp. */
  public RunWinches(double mainWinchSpeed, double adjWinchSpeed, WinchSubsystem winchSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_winchSubsystem = winchSubsystem;
    mainSpeed = mainWinchSpeed;
    adjSpeed = adjWinchSpeed;

    addRequirements(winchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_winchSubsystem.stopAdjustmentWinch();
    m_winchSubsystem.stopMainWinch();

    System.out.println("WinchSubsystem: Started winching | main winch speed: " + mainSpeed + " | adjustment winch speed: " + adjSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_winchSubsystem.setWinches(adjSpeed, mainSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_winchSubsystem.stopAdjustmentWinch();
    m_winchSubsystem.stopMainWinch();

    System.out.println("WinchSubsystem: Finished winching");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
