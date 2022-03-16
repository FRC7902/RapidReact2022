package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase{
    private final CANSparkMax m_ropeOne = new CANSparkMax(WinchConstants.kMainWinchCAN, MotorType.kBrushless);
    private final WPI_VictorSPX m_ropeTwo = new WPI_VictorSPX(WinchConstants.kAdjustmentWinchCAN);

    private String mainWinchStatus = "Off";
    private String adjWinchStatus = "Off";

    public WinchSubsystem() {
        m_ropeOne.restoreFactoryDefaults();
        m_ropeOne.setSmartCurrentLimit(WinchConstants.kCurrentLimit);
        m_ropeOne.setInverted(false);
        m_ropeTwo.setInverted(false);
    }

    public void setWinches(double mainWinch, double adjWinch){
        setMainWinch(mainWinch);
        setAdjustmentWinch(adjWinch);
    }

    public void setMainWinch(double power) {
        m_ropeOne.set(power);

        if (power > 0) {
            mainWinchStatus = "Winching up...";
        } else if (power < 0) {
            mainWinchStatus = "Winching down...";
        } else {
            mainWinchStatus = "Off";
        }
    }

    public void setAdjustmentWinch(double power) {
        m_ropeTwo.set(power);

        if (power > 0) {
            adjWinchStatus = "Winching up...";
        } else if (power < 0) {
            adjWinchStatus = "Winching down...";
        } else {
            adjWinchStatus = "Off";
        }
    }

    public void stopMainWinch() {
        m_ropeOne.stopMotor();
        mainWinchStatus = "Off";
    }

    public void stopAdjustmentWinch() {
        m_ropeTwo.stopMotor();
        adjWinchStatus = "Off";
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimbSubsystem/Rope 1 Power", m_ropeOne.getAppliedOutput());
        SmartDashboard.putNumber("ClimbSubsystem/Rope 2 Power", m_ropeTwo.getMotorOutputPercent());

        SmartDashboard.putString("ClimbSubsystem/Main Winch Status", mainWinchStatus);
        SmartDashboard.putString("ClimbSubsystem/Adjustment Winch Status", adjWinchStatus);
    }
}
