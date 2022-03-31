package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ExtendElevator extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;

    public ExtendElevator(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.stopElevator();
        System.out.println("ElevatorSubsystem: Started extending elevator");
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setElevator(Constants.ElevatorConstants.kExtendElevatorSpeed);
    }


    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.stopElevator();
        System.out.println("ElevatorSubsystem: Finished extending elevator");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
