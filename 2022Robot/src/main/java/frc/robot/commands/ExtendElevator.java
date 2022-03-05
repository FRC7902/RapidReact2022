package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class ExtendElevator extends CommandBase {
    private final ClimbSubsystem m_climbSubsystem;

    public ExtendElevator(ClimbSubsystem climbSubsystem) {
        m_climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void execute() {
        m_climbSubsystem.setElevator(Constants.ClimbConstants.kExtendElevatorSpeed);
    }

    @Override
    public void initialize() {
        m_climbSubsystem.stopElevator();
    }

    @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
