// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorToHeightPID extends PIDCommand {
  private ElevatorSubsystem m_climbSubsystem;
  /** Creates a new ExtendElevatorToHeight. */
  public SetElevatorToHeightPID(double setHeight, ElevatorSubsystem climbSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.5, 0, 0),
        // This should return the measurement
        () -> climbSubsystem.getElevatorRotations(),
        // This should return the setpoint (can also be a constant)
        setHeight,
        // This uses the output
        output -> {
          // Use the output here
          climbSubsystem.setElevator(output);
        });

    m_climbSubsystem = climbSubsystem;
    getController().setTolerance(1, 0.1);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted){
    m_climbSubsystem.stopElevator();
  }
}
