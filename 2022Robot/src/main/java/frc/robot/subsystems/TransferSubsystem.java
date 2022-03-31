// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {

  public final WPI_VictorSPX transferMotor = new WPI_VictorSPX(Constants.TransferConstants.kVertTransferCAN);
  

  public String status = "Off";

  /** Creates a new TransferSubsystem. */
  public TransferSubsystem() {
    transferMotor.setInverted(false);

    transferMotor.configOpenloopRamp(Constants.TransferConstants.kRampTime);
  }


  public void setSpeed(double speed){
    transferMotor.set(speed);
    if(speed > 0){
      status = "Transferring up...";
    }else if(speed < 0){
      status = "Transferring down...";
    }
  }


  public void stop(){
    transferMotor.stopMotor();
    status = "Off";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CompetitionView/Vertical Transfer Power", transferMotor.getMotorOutputPercent());
    SmartDashboard.putString("CompetitionView/Transfer Status", status);

    SmartDashboard.putNumber("TransferSubsystem/Vertical Transfer Power", transferMotor.getMotorOutputPercent());
    SmartDashboard.putString("TransferSubsystem/Transfer Status", status);
  }
}
