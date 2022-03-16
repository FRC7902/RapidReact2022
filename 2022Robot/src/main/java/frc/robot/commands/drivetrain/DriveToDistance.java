// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToDistance extends PIDCommand {

  private final DriveSubsystem m_driveSubsystem;
  /** Creates a new DriveToDistance. */
  public DriveToDistance(double targetDist, DriveSubsystem driveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        driveSubsystem::getAvgEncoderDistance,
        // This should return the setpoint (can also be a constant)
        targetDist,
        // This uses the output
        output -> {
          // Use the output here
          driveSubsystem.driveRaw(output, output);
        }, driveSubsystem);
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    getController().setTolerance(1, 0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted){
    // m_driveSubsystem.refreshOdometry();
    m_driveSubsystem.resetEncoders();
    // m_driveSubsystem.resetEncoderSims();
  }
}
