// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  //SpeedController Groups
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(new PWMVictorSPX(Constants.DriveConstants.FL), new PWMVictorSPX(Constants.DriveConstants.BL));
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(new PWMVictorSPX(Constants.DriveConstants.FR), new PWMVictorSPX(Constants.DriveConstants.BR));

  //Encoders
  private final Encoder m_leftEncoders = new Encoder(Constants.DriveConstants.kLeftEnc1, Constants.DriveConstants.kLeftEnc2, Constants.DriveConstants.kLeftEnc3, true);
  private final Encoder m_rightEncoders = new Encoder(Constants.DriveConstants.kRightEnc1, Constants.DriveConstants.kRightEnc2, Constants.DriveConstants.kRightEnc3, true);

  //Gyro
  private final AnalogGyro m_gyro = new AnalogGyro(1);

  //Drive
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


  //Simulation Stuff
  private final DifferentialDriveOdometry m_odometry;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private Field2d m_fieldSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //Set Encoder pulses
    m_leftEncoders.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoders.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    resetEncoders();


    //Set up robot simulation
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), new Pose2d(4, 5, new Rotation2d())); 
    m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDualCIMPerSide, 
    KitbotGearing.k10p71, 
    KitbotWheelSize.SixInch, null);
    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);

    //Connect the simulators with their counterparts
    m_leftEncoderSim = new EncoderSim(m_leftEncoders);
    m_rightEncoderSim = new EncoderSim(m_rightEncoders);
    m_gyroSim = new AnalogGyroSim(m_gyro);

    //Invert the speed controller groups
    m_leftMotors.setInverted(true);
    m_rightMotors.setInverted(true);    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //update the position of the robot
    m_odometry.update( 
      Rotation2d.fromDegrees(getHeading()),
      m_leftEncoders.getDistance(),
      m_rightEncoders.getDistance()
    );
    m_fieldSim.setRobotPose(getPose());


    SmartDashboard.putNumber("Enc Distance", getAvgEncoderDistance());
    SmartDashboard.putNumber("Gyro Heading", getHeading());

  }




  @Override
  public void simulationPeriodic() {

    //connect the motors to update the drivetrain
    m_driveTrainSim.setInputs(
      m_leftMotors.get() * RobotController.getBatteryVoltage(), 
      m_rightMotors.get() * RobotController.getBatteryVoltage()
    );

    //Run and update simulation
    m_driveTrainSim.update(0.02);
    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());



  }



  //Get the current draw from the drivetrain
  public double getDrawnCurrentAmps(){
    return m_driveTrainSim.getCurrentDrawAmps();
  }

  //Get the position of the robot
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  //get the wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_leftEncoders.getRate(), m_rightEncoders.getRate());
  }

  //reset the robots position
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_driveTrainSim.setPose(pose);
    m_odometry.resetPosition(pose, new Rotation2d());

  }


  //drive the robot using arcade drive
  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot);
  }


  //stop the motors
  public void stopMotors(){
    m_leftMotors.stopMotor();
    m_rightMotors.stopMotor();
  }

  //drive the robot
  public void drive(double x, double y){
    m_leftMotors.set(-y + x * Constants.DriveConstants.kTurnSpeed);
    m_rightMotors.set(-y-x * Constants.DriveConstants.kTurnSpeed);

  }

  //set the motors to a specific speed
  public void driveRaw(double left, double right){
    m_leftMotors.set(left);
    m_rightMotors.set(right);
  }

  //reset the encoders
  public void resetEncoders(){
    m_leftEncoders.reset();
    m_rightEncoders.reset();
  }



  //get the robot distance
  public double getAvgEncoderDistance(){
    return (m_leftEncoders.getDistance() + m_rightEncoders.getDistance()) / 2.0;
  }


  //return left encoder
  public Encoder getLeftEncoder(){
    return m_leftEncoders;
  }

  //return right encoder
  public Encoder getRightEncoder(){
    return m_rightEncoders;
  }

  //reset the gyro
  public void zeroHeading(){
    m_gyro.reset();
  }

  //get the heading angle from the gyro
  public double getHeading(){
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

}
