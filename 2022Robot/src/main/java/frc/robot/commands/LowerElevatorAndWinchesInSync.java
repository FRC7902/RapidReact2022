// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.RetractElevator;
import frc.robot.commands.winches.RunWinches;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WinchSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowerElevatorAndWinchesInSync extends ParallelCommandGroup {
  /** Creates a new LowerElevatorAndWindWinches. */
  public LowerElevatorAndWinchesInSync(ElevatorSubsystem elevatorSubsystem, WinchSubsystem winchSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RetractElevator(elevatorSubsystem),
      new RunWinches(Constants.WinchConstants.kSyncMainWinchInSpeed, Constants.WinchConstants.kSyncAdjWinchInSpeed, winchSubsystem)
    );
  }
}
