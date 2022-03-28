// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoclimb;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.TimedDriveWithSpeed;
import frc.robot.commands.elevator.ExtendElevator;
import frc.robot.commands.elevator.RetractElevator;
import frc.robot.commands.winches.RunWinches;
import frc.robot.commands.winches.WinchIn;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WinchSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHighStage2 extends SequentialCommandGroup {
  /** Creates a new AutoHighStage2. */
  public AutoHighStage2(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, WinchSubsystem winchSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RetractElevator(elevatorSubsystem).withTimeout(1.8),
      new TimedDriveWithSpeed(-0.5, 0.25, driveSubsystem),
      new WinchIn(winchSubsystem).withTimeout(4.3),
      new RunWinches(0, 0.5, winchSubsystem).withTimeout(0.6),
      new ExtendElevator(elevatorSubsystem).withTimeout(1)
    );

  }
}
