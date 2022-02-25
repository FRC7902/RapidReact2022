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

        public static final int kLeftLeaderCAN = 4;
        public static final int kLeftFollowerCAN = 3;
        public static final int kRightLeaderCAN = 12;
        public static final int kRightFollowerCAN = 11;

        public static final int[] kLeftEncoderIDs = {0, 1};
        public static final int[] kRightEncoderIDs = {3, 4};

        public static final int kGyroCAN = 1;

        //Sensitivities (1 -> quadratic, 0 -> linear)
        public static final double kForwardSens = 1;
        public static final double kTurnSens = 0.5;
        public static final double kTurnMax = 1;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    }

    public final static class IntakeConstants {
        public static final int kIntakePowerCAN = 13;
        public static final int kIntakeDeplCAN = 10;

        public final static double suckSpeed = 0.7;
        public final static double spitSpeed = -0.5;

    }

    public final static class TransferConstants {

        public static final int kHoriTransferCAN = 6;
        public static final int kVertTransferCAN = 5;

        public final static double horiForwardSpeed = 0.7;
        public final static double horiBackwardsSpeed = -0.5;


        public final static double vertForwardSpeed = 1;
        public final static double vertBackwardsSpeed = -0.4;
    }
    
    public final static class ShooterConstants {
        public static final int kMasterCAN = 8;
        public static final int kFollowerCAN = 9;
        
        public final static double kSpeed = 1;
    }
}
