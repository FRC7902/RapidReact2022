// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.winches;

import frc.robot.Constants;
import frc.robot.subsystems.WinchSubsystem;

/** Add your docs here. */
public class SyncWinchIn extends RunWinches{
    public SyncWinchIn(WinchSubsystem winchSubsystem){
        super (Constants.WinchConstants.kSyncMainWinchInSpeed, Constants.WinchConstants.kSyncAdjWinchInSpeed, winchSubsystem);
        addRequirements(winchSubsystem);
    }
}
