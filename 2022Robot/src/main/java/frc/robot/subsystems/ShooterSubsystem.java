// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public final WPI_VictorSPX left = new WPI_VictorSPX(8);
  public final WPI_VictorSPX right = new WPI_VictorSPX(9);


  public ShooterSubsystem() {
    
    left.setInverted(true);
    right.setInverted(false);

  }

  public void shoot() {
    left.set(Constants.ShooterConstants.kSpeed);
    right.set(Constants.ShooterConstants.kSpeed);
  }

  public void stop() {
    left.stopMotor();
    right.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
