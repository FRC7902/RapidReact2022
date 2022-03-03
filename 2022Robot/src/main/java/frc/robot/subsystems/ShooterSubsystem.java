// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public final WPI_VictorSPX master = new WPI_VictorSPX(Constants.ShooterConstants.kMasterCAN);// left
  public final WPI_VictorSPX follower = new WPI_VictorSPX(Constants.ShooterConstants.kFollowerCAN); //right

  public double shooterSpeed;
  public String status = "Off";

  public ShooterSubsystem() {
    follower.follow(master);
    follower.setInverted(InvertType.FollowMaster);

    shooterSpeed = Constants.ShooterConstants.kSpeed;

    SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
    


    
    
    master.setInverted(false);
    // right.setInverted(false);

  }

  public void shoot() {
    master.set(shooterSpeed);
    // right.set(Constants.ShooterConstants.kSpeed);
    status = "Shooting...";
  }

  public void stop() {
    master.stopMotor();
    // right.stopMotor();
    status = "Off";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power", master.getMotorOutputVoltage());
    SmartDashboard.putString("ShooterSubsystem/Shooter Status", status);

    SmartDashboard.putNumber("CompetitionView/Shooter Power", master.getMotorOutputVoltage());
    SmartDashboard.putString("CompetitionView/Shooter Status", status);
    // SmartDashboard.putNumber("Right Shooter", follower.getMotorOutputVoltage());

    shooterSpeed = SmartDashboard.getNumber("Shooter Speed", Constants.ShooterConstants.kSpeed);
  }
}


