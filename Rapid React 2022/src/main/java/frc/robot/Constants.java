package frc.robot;

public class Constants {

    public static final class DriveConstants {
        // Drive Controllers
        public static final int FL = 4, 
            FR = 9, 
            BL = 3, 
            BR = 8;

        // Drive Limiters
        public static final double kLimit = 1;
        public static final double kDriveSpeed = 1;
        public static final double kTurnSpeed = 0.9;
        // Travel Speed
        public static final double kAutoSpeed = 0.75;
        // Turn Speed
        public static final double kAutoTurnSpeed = 0.5;
        // Avoid Zero Error 
        public static final double kNoZero = 0.001;

        // Diameter of Robot in meters
        public static final double kRobotDiameter = 0.582;

        // Encoder
        public static final int kLeftMotor1Port = 0,
            kLeftMotor2Port = 1,
            kLeftMotor3Port = 2,
            kRightMotor1Port = 3,
            kRightMotor2Port = 4,
            kRightMotor3Port = 5;
        // Maps Pulse to Distance (m/pulse)
        public static final double kEncoderDistancePerPulse = 1.0 / 2048.0 * Math.PI * 0.1524;
        // Random Error Avg Number
        public static final int kAvgNum = 5;
        // Minimum Rate Cap (m/s)
        public static final double kMinRate = 1.0;
    }
}
