// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {

  public final WPI_VictorSPX horiTransfer = new WPI_VictorSPX(Constants.TransferConstants.kHoriTransferCAN);
  public final WPI_VictorSPX vertTransfer = new WPI_VictorSPX(Constants.TransferConstants.kVertTransferCAN);
  

  public String status = "Off";

  /** Creates a new TransferSubsystem. */
  public TransferSubsystem() {
    horiTransfer.setInverted(false);
    vertTransfer.setInverted(false);
  }

  public void transfer(){
    horiTransfer.set(Constants.TransferConstants.horiForwardSpeed);
    vertTransfer.set(Constants.TransferConstants.vertForwardSpeed);
    status = "Transferring...";
  }

  public void reverse(){
    horiTransfer.set(Constants.TransferConstants.horiBackwardsSpeed);
    vertTransfer.set(Constants.TransferConstants.vertBackwardsSpeed);
    status = "Reversing...";
  }

  public void stopTransfer(){
    horiTransfer.stopMotor();
    vertTransfer.stopMotor();
    status = "Off";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CompetitionView/Horizontal Transfer Power", horiTransfer.getMotorOutputPercent());
    SmartDashboard.putNumber("CompetitionView/Vertical Transfer Power", vertTransfer.getMotorOutputPercent());
    SmartDashboard.putString("CompetitionView/Transfer Status", status);

    SmartDashboard.putNumber("TransferSubsystem/Horizontal Transfer Power", horiTransfer.getMotorOutputPercent());
    SmartDashboard.putNumber("TransferSubsystem/Vertical Transfer Power", vertTransfer.getMotorOutputPercent());
    SmartDashboard.putString("TransferSubsystem/Transfer Status", status);
  }
}
