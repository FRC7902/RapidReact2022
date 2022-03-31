package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class RetractElevator extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    public RetractElevator(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setElevator(Constants.ElevatorConstants.kRetractElevatorSpeed);
        
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.stopElevator();

        System.out.println("ElevatorSubsystem: Started retracting elevator");
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.stopElevator();

        System.out.println("ElevatorSubsystem: Finished retracting elevator");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
