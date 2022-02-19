// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {

  public final WPI_VictorSPX horiTransfer = new WPI_VictorSPX(6);
  public final WPI_VictorSPX vertTransfer = new WPI_VictorSPX(5);

  /** Creates a new TransferSubsystem. */
  public TransferSubsystem() {
    horiTransfer.setInverted(false);
    vertTransfer.setInverted(false);
  }

  public void transfer(){
    horiTransfer.set(Constants.TransferConstants.horiForwardSpeed);
    vertTransfer.set(Constants.TransferConstants.vertForwardSpeed);
  }

  public void reverse(){
    horiTransfer.set(Constants.TransferConstants.horiBackwardsSpeed);
    vertTransfer.set(Constants.TransferConstants.vertBackwardsSpeed);
  }

  public void stopTransfer(){
    horiTransfer.stopMotor();
    vertTransfer.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
