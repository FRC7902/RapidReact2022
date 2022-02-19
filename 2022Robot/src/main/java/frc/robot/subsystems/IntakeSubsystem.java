// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public final WPI_VictorSPX intakePower = new WPI_VictorSPX(13);
  public final WPI_VictorSPX intakeDepl = new WPI_VictorSPX(10);


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakePower.setInverted(false);
    intakeDepl.setInverted(false);

  }

  public void suck(){
    intakePower.set(Constants.IntakeConstants.suckSpeed);
  }

  public void spit(){
    intakePower.set(Constants.IntakeConstants.spitSpeed);
  }

  public void setIntakeArm(double power){
    intakeDepl.set(power);
  }

  public void stopIntakePower(){
    intakePower.stopMotor();
  }

  public void stopIntakeArm(){
    intakeDepl.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
