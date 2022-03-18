// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoclimb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RaiseElevatorAndWinchesInSync;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WinchSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHighStage1 extends SequentialCommandGroup {
  /** Creates a new AutoHighStage1. */
  public AutoHighStage1(ElevatorSubsystem elevatorSubsystem, WinchSubsystem winchSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RaiseElevatorAndWinchesInSync(elevatorSubsystem, winchSubsystem).withTimeout(4)
    );
  }
}
