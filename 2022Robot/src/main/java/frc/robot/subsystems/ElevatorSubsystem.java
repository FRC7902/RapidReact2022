package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax m_elevator = new CANSparkMax(ElevatorConstants.kElevatorCAN, MotorType.kBrushless);
    private RelativeEncoder m_encoder;

    private String elevatorStatus = "Off";

    public ElevatorSubsystem() {
        m_elevator.restoreFactoryDefaults();

        m_elevator.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);

        m_elevator.setInverted(true);
        
        m_encoder = m_elevator.getEncoder();
    }

    public double getElevatorRotations(){
        return m_encoder.getPosition();
    }

    public void resetEncoder(){
        m_encoder.setPosition(0);
    }

    public void setElevator(double power) {
        m_elevator.set(power);

        if(power > 0){
            elevatorStatus = "Rising...";
        }else if(power < 0){
            elevatorStatus = "Lowering...";
        }else{
            elevatorStatus = "Off";
        }

    }

    public void stopElevator() {
        m_elevator.set(0);
        m_elevator.stopMotor();
        
        elevatorStatus = "Off";
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ClimbSubsystem/Elevator Power", m_elevator.getAppliedOutput());
        SmartDashboard.putNumber("ClimbSubsystem/Encoder Reading", getElevatorRotations());

        SmartDashboard.putString("ClimbSubsystem/Elevator Status", elevatorStatus);
    }
}
