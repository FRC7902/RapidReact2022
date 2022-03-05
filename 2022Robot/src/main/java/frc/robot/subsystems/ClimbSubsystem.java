package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax m_elevator = new CANSparkMax(ClimbConstants.kElevatorPort, MotorType.kBrushless);
    private final CANSparkMax m_ropeOne = new CANSparkMax(ClimbConstants.kMainWinchPort, MotorType.kBrushless);
    private final WPI_VictorSPX m_ropeTwo = new WPI_VictorSPX(ClimbConstants.kAdjustmentWinchPort);

    public ClimbSubsystem() {

    }

    public void setElevator(double power) {
        m_elevator.set(power);
    }

    public void setMainWinch(double power) {
        m_ropeOne.set(power);
    }

    public void setAdjustmentWinch(double power) {
        m_ropeTwo.set(power);
    }

    public void stopElevator() {
        m_elevator.stopMotor();
    }

    public void stopMainWinch() {
        m_ropeOne.stopMotor();
    }

    public void stopAdjustmentWinch() {
        m_ropeTwo.stopMotor();
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("ClimbSubsystem/Elevator Power", m_elevator.get());

        SmartDashboard.putNumber("ClimbSubsystem/Elevator Power", m_elevator.getAppliedOutput());
        SmartDashboard.putNumber("ClimbSubsystem/Rope 1 Power", m_ropeOne.getAppliedOutput());
        SmartDashboard.putNumber("ClimbSubsystem/Rope 2 Power", m_ropeTwo.getMotorOutputPercent());
        SmartDashboard.putNumber("CompetitionView/Elevator Power", m_elevator.getAppliedOutput());
        SmartDashboard.putNumber("CompetitionView/Rope 1 Power", m_ropeOne.getAppliedOutput());
        SmartDashboard.putNumber("CompetitionView/Rope 2 Power", m_ropeTwo.getMotorOutputPercent());

    }
    
}
