// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public final WPI_VictorSPX intakePower = new WPI_VictorSPX(Constants.IntakeConstants.kIntakePowerCAN);
  public final WPI_VictorSPX intakeDepl = new WPI_VictorSPX(Constants.IntakeConstants.kIntakeDeplCAN);

  public String status = "Off";


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakePower.setInverted(false);
    intakeDepl.setInverted(false);

  }

  public void suck(){
    intakePower.set(Constants.IntakeConstants.suckSpeed);
    status = "Sucking...";
  }

  public void spit(){
    intakePower.set(Constants.IntakeConstants.spitSpeed);
    status = "Spitting...";
  }

  public void setIntakeArm(double power){
    intakeDepl.set(power);
  }

  public void stopIntakePower(){
    intakePower.stopMotor();
    status = "Off";
  }

  public void stopIntakeArm(){
    intakeDepl.stopMotor();
    // intakeDepl.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CompetitionView/Intake Power", intakePower.getMotorOutputPercent());
    SmartDashboard.putNumber("CompetitionView/Intake Arm Power", intakeDepl.getMotorOutputPercent());
    SmartDashboard.putString("CompetitionView/Intake Status", status);

    SmartDashboard.putNumber("IntakeSubsystem/Intake Power", intakePower.getMotorOutputPercent());
    SmartDashboard.putNumber("IntakeSubsystem/Intake Arm Power", intakeDepl.getMotorOutputPercent());
    SmartDashboard.putString("IntakeSubsystem/Intake Status", status);
  }
}
