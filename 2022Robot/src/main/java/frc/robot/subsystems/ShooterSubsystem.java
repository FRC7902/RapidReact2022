// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public final WPI_VictorSPX master = new WPI_VictorSPX(Constants.ShooterConstants.kMasterCAN);// left
  public final WPI_VictorSPX follower = new WPI_VictorSPX(Constants.ShooterConstants.kFollowerCAN); //right

  public String status = "Off";

  public ShooterSubsystem() {

    

    follower.follow(master);

    master.setInverted(false);
    follower.setInverted(InvertType.FollowMaster);

    master.configOpenloopRamp(Constants.ShooterConstants.kRampTime);
    follower.configOpenloopRamp(Constants.ShooterConstants.kRampTime);

    master.setNeutralMode(NeutralMode.Brake);
    follower.setNeutralMode(NeutralMode.Brake);

  }

  public void setSpeed(double speed) {
    master.set(speed);
    if(speed > 0){
      status = "Shooting...";
    }else if(speed < 0){
      status = "Reversing...";
    }
  }

  public void stop() {
    master.stopMotor();
    master.set(-0.04);
    status = "Off";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power", master.getMotorOutputPercent());
    SmartDashboard.putNumber("ShooterSubsystem/Shooter Power 2", follower.getMotorOutputPercent());
    SmartDashboard.putString("ShooterSubsystem/Shooter Status", status);

    SmartDashboard.putNumber("CompetitionView/Shooter Power", master.getMotorOutputPercent());
    SmartDashboard.putString("CompetitionView/Shooter Status", status);

  }
}


