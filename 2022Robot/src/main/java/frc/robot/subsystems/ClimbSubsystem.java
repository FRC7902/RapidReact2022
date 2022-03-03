package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax m_elevator = new CANSparkMax(ClimbConstants.kElevatorPort, MotorType.kBrushless);
    private final CANSparkMax m_ropeOne = new CANSparkMax(ClimbConstants.kMainWinchPort, MotorType.kBrushless);
    private final WPI_VictorSPX m_ropeTwo = new WPI_VictorSPX(ClimbConstants.kAdjustmentWinchPort);
    private RelativeEncoder m_encoder;

    private String elevatorStatus = "Off";
    private String mainWinchStatus = "Off";
    private String adjWinchStatus = "Off";



    public ClimbSubsystem() {
        m_elevator.restoreFactoryDefaults();
        m_ropeOne.restoreFactoryDefaults();

        m_encoder = m_elevator.getEncoder();
        m_elevator.setInverted(false);
        m_ropeOne.setInverted(false);
        m_ropeTwo.setInverted(false);
    }

    public double getElevatorRotations(){
        return m_encoder.getPosition();
    }

    public void resetEncoder(){
        m_encoder.setPosition(0);
    }

    public void setWinches(double mainWinch, double adjWinch){
        setMainWinch(mainWinch);
        setAdjustmentWinch(adjWinch);
    }

    public void setElevator(double power) {
        m_elevator.set(power);

        elevatorStatus = (power > 0 ? "Rising" : "Lowering");
    }

    public void setMainWinch(double power) {
        m_ropeOne.set(power);
        mainWinchStatus = (power > 0 ? "Winching up..." : "Winching down...");
        
    }

    public void setAdjustmentWinch(double power) {
        m_ropeTwo.set(power);
        adjWinchStatus = (power > 0 ? "Winching up..." : "Winching down...");
    }

    public void stopElevator() {
        m_elevator.stopMotor();
        elevatorStatus = "Off";
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
    public void periodic(){

        SmartDashboard.putNumber("ClimbSubsystem/Elevator Power", m_elevator.getAppliedOutput());
        SmartDashboard.putNumber("ClimbSubsystem/Rope 1 Power", m_ropeOne.getAppliedOutput());
        SmartDashboard.putNumber("ClimbSubsystem/Rope 2 Power", m_ropeTwo.getMotorOutputPercent());
        SmartDashboard.putNumber("ClimbSubsystem/Encoder Reading", getElevatorRotations());

        SmartDashboard.putString("ClimbSubsystem/Elevator Status", elevatorStatus);
        SmartDashboard.putString("ClimbSubsystem/Main Winch Status", mainWinchStatus);
        SmartDashboard.putString("ClimbSubsystem/Adjustment Winch Status", adjWinchStatus);

        SmartDashboard.putNumber("CompetitionView/Elevator Power", m_elevator.getAppliedOutput());
        SmartDashboard.putNumber("CompetitionView/Rope 1 Power", m_ropeOne.getAppliedOutput());
        SmartDashboard.putNumber("CompetitionView/Rope 2 Power", m_ropeTwo.getMotorOutputPercent());
        SmartDashboard.putNumber("CompetitionView/Encoder Reading", getElevatorRotations());

        SmartDashboard.putString("CompetitionView/Elevator Status", elevatorStatus);
        SmartDashboard.putString("CompetitionView/Main Winch Status", mainWinchStatus);
        SmartDashboard.putString("CompetitionView/Adjustment Winch Status", adjWinchStatus);
    }
    
}
