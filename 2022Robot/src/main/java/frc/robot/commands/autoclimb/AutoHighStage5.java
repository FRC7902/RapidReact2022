// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoclimb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.winches.RollBackwards;
import frc.robot.commands.winches.RollForward;
import frc.robot.commands.winches.RunWinches;
import frc.robot.subsystems.WinchSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHighStage5 extends SequentialCommandGroup {
  /** Creates a new AutoHighStage5. */
  public AutoHighStage5(WinchSubsystem winchSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RollForward(winchSubsystem).withTimeout(1)
    );
  }
}
