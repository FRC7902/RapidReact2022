// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.winches;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WinchSubsystem;

public class WinchOut extends RunWinches{
  /** Creates a new WinchUp. */
  public WinchOut(WinchSubsystem winchSubsystem) {
    super(Constants.WinchConstants.kMainWinchOutSpeed, Constants.WinchConstants.kAdjWinchOutSpeed / 2, winchSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(winchSubsystem);
  }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {}

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
