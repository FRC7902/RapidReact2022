// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorToHeight extends CommandBase {
  private ElevatorSubsystem m_climbSubsystem;
  private double setHeight;
  /** Creates a new SetElevatorToHeight. */
  public SetElevatorToHeight(double setHeight, ElevatorSubsystem climbSubsystem) {
    m_climbSubsystem = climbSubsystem;
    this.setHeight = setHeight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climbSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.stopElevator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_climbSubsystem.getElevatorRotations() > setHeight){
      m_climbSubsystem.setElevator(-0.5);
    }else{
      m_climbSubsystem.setElevator(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return setHeight + 0.5 >= m_climbSubsystem.getElevatorRotations() && setHeight-0.5 <= m_climbSubsystem.getElevatorRotations();
  }
}
