package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase{
    private final CANSparkMax mainWinch = new CANSparkMax(WinchConstants.kMainWinchCAN, MotorType.kBrushless);
    private final WPI_VictorSPX adjWinch = new WPI_VictorSPX(WinchConstants.kAdjustmentWinchCAN);

    private String mainWinchStatus = "Off";
    private String adjWinchStatus = "Off";

    public WinchSubsystem() {
        mainWinch.restoreFactoryDefaults();
        mainWinch.setSmartCurrentLimit(WinchConstants.kCurrentLimit);
        mainWinch.setInverted(true);
        adjWinch.setInverted(false);
    }

    public void setWinches(double adjWinchSpeed, double mainWinchSpeed){
        setMainWinch(mainWinchSpeed);
        setAdjustmentWinch(adjWinchSpeed);
    }

    public void setMainWinch(double power) {
        mainWinch.set(power);

        if (power > 0) {
            mainWinchStatus = "Winching up...";
        } else if (power < 0) {
            mainWinchStatus = "Winching down...";
        } else {
            mainWinchStatus = "Off";
        }
    }

    public void setAdjustmentWinch(double power) {
        adjWinch.set(power);

        if (power > 0) {
            adjWinchStatus = "Winching up...";
        } else if (power < 0) {
            adjWinchStatus = "Winching down...";
        } else {
            adjWinchStatus = "Off";
        }
    }

    public void stopMainWinch() {
        mainWinch.stopMotor();
        mainWinchStatus = "Off";
    }

    public void stopAdjustmentWinch() {
        adjWinch.stopMotor();
        adjWinchStatus = "Off";
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimbSubsystem/Rope 1 Power", mainWinch.getAppliedOutput());
        SmartDashboard.putNumber("ClimbSubsystem/Rope 2 Power", adjWinch.getMotorOutputPercent());

        SmartDashboard.putString("ClimbSubsystem/Main Winch Status", mainWinchStatus);
        SmartDashboard.putString("ClimbSubsystem/Adjustment Winch Status", adjWinchStatus);
    }
}
