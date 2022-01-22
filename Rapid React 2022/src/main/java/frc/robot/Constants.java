package frc.robot;

public class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 3;
        public static final int kLeftMotor2Port = 4;
        public static final int kRightMotor1Port = 8;
        public static final int kRightMotor2Port = 9;


        public static final int[] kLeftEncoderPorts = {0, 1, 2};
        public static final int[] kRightEncoderPorts = {3, 4, 5};

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;


        public static final double kTurnSpeed = 0.9;
        public static final double kSlowSpeed = 0.5;
    }

    // Joystick USB Slot
    public static final int JOY = 0,
    OP = 1;

    // Joystick - Button
    public static final int A = 1,
    B = 2,
    X = 3,
    Y = 4,
    LB = 5, // Bumper
    RB = 6, // Bumper
    M = 7, // menu
    S = 8, // start
    LA = 9, // Press left axis
    RA = 10; // Press right axis

    // Joystick - Axis
    public static final int LX = 0,
    LY = 1,
    LT = 2, // Trigger
    RT = 3, // Trigger
    RX = 4,
    RY = 5, 
    PX = 6, // D-Pad
    PY = 7; // D-Pad
}
