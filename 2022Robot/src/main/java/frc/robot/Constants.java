// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public final static class DriveConstants {

        public static final int kLeftLeaderCAN = 11;
        public static final int kLeftFollowerCAN = 12;
        public static final int kRightLeaderCAN = 4;
        public static final int kRightFollowerCAN = 3;

        public static final int[] kLeftEncoderIDs = {0, 1};
        public static final int[] kRightEncoderIDs = {3, 4};

        public static final int kGyroCAN = 1;

        //Sensitivities (1 -> quadratic, 0 -> linear)
        public static final double kForwardSens = 0;
        public static final double kTurnSens = 0;
        public static final double kTurnMax = 0.6;

        public static final double kDriveSlowSpeed = 0.1;
        public static final double kTurnSlowSpeed = 0.1;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kRampTime = 0.1;
        public static final int kCurrentLimit = 10;

        public static final double kDeadzoneY = 0.01;
        public static final double kDeadzoneX = 0.01;

    }

    public final static class IntakeConstants {
        public static final int kIntakePowerCAN = 13;
        public static final int kIntakeDeplCAN = 10;

        public final static double kSuckSpeed = 0.54;
        public final static double kSpitSpeed = -0.54;

        public final static double kPowerRampTime = 0;
        public final static double kDeplRampTime = 0;

    }

    public final static class TransferConstants {

        public static final int kVertTransferCAN = 5;

        public final static double kVertForwardSpeed = 0.75;
        public final static double kVertBackwardsSpeed = -0.75;

        public final static double kRampTime = 0;
    }
    
    public final static class ShooterConstants {
        public static final int kMasterCAN = 8;
        public static final int kFollowerCAN = 9;
        
        public final static double kHighSpeed = 1;
        public final static double kLowSpeed = 0.62;

        public final static double kRampTime = 0;
    }
    
    public static final class ElevatorConstants {
        public static final int kElevatorCAN = 1;

        public static final double kExtendElevatorSpeed = 0.7;
        public static final double kRetractElevatorSpeed = -0.7;
        
        public static final int kCurrentLimit = 35;
    }

    public static final class WinchConstants {
        public static final int kMainWinchCAN = 14;
        public static final int kAdjustmentWinchCAN = 2;
        public static final int kCurrentLimit = 35;


        //Speeds for just joystick control
        public static final double kMainWinchInSpeed = 0.5;
        public static final double kMainWinchOutSpeed = -0.5;
        public static final double kAdjWinchInSpeed = 0.5;
        public static final double kAdjWinchOutSpeed = -0.5;


        //Speeds in order to be synchronized with elevator
        public static final double kSyncMainWinchInSpeed = -0.3;
        public static final double kSyncMainWinchOutSpeed = 0.3;
        public static final double kSyncAdjWinchInSpeed = -0.3;
        public static final double kSyncAdjWinchOutSpeed = 0.3;
    }

    public static final class IOConstants{
        public static final int kDriverStick = 0;
        public static final int kClimbStick = 1;

        // Joystick Buttons
        public static final int kA = 1,
                                kB = 2,
                                kX = 3,
                                kY = 4,
                                kLB = 5,
                                kRB = 6,
                                kMENU = 7,
                                kSTART = 8,
                                kLA = 9,
                                kRA = 10;

        // Joystick Axis
        public static final int kLX = 0,
                                kLY = 1,
                                kLT = 2,
                                kRT = 3,
                                kRX = 4,
                                kRY = 5,
                                kDX = 6,
                                kDY = 7;


    }
  
}
