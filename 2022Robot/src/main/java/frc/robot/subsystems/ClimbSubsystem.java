package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final PWMSparkMax m_elevator = new PWMSparkMax(ClimbConstants.kElevatorPort);
    private final PWMSparkMax m_ropeOne = new PWMSparkMax(ClimbConstants.kMainWinchPort);
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
}
