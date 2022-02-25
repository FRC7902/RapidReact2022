// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  //Motor Controllers
  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(Constants.DriveConstants.kLeftLeaderCAN);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(Constants.DriveConstants.kLeftFollowerCAN);
  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(Constants.DriveConstants.kRightLeaderCAN);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(Constants.DriveConstants.kRightFollowerCAN);
  
  //Encoders
  private final Encoder m_leftEncoder = new Encoder(Constants.DriveConstants.kLeftEncoderIDs[0], Constants.DriveConstants.kLeftEncoderIDs[1]);
  private final Encoder m_rightEncoder = new Encoder(Constants.DriveConstants.kRightEncoderIDs[0], Constants.DriveConstants.kRightEncoderIDs[1]);

  //Gyro (Pigeon)
  private final WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(Constants.DriveConstants.kGyroCAN);

  //Slow State Booleans
  private boolean isForwardSlow = false;
  private boolean isTurnSlow = false;  


  //SIMULATION

  //Talon simulators
  private TalonSRXSimCollection m_leftDriveSim;
  private TalonSRXSimCollection m_rightDriveSim;

  //Pigeon simulators
  private BasePigeonSimCollection m_pigeonSim;

  //Encoder simulators
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  

  //Other Simulation Classes
  private DifferentialDrivetrainSim m_drivetrainSim;
  private Field2d m_fieldSim;
  private DifferentialDriveOdometry m_odometry;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    //Encoder config
    m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    resetEncoders();

    //Motor controller config
    m_rightLeader.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_rightFollower.follow(m_rightLeader);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    m_leftLeader.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_leftFollower.follow(m_leftLeader);
    m_leftFollower.setInverted(InvertType.FollowMaster);

    m_leftLeader.setInverted(false);
    m_leftLeader.setSensorPhase(false);

    m_rightLeader.setInverted(true);
    m_rightLeader.setSensorPhase(false);


    //SIMULATION

    if(RobotBase.isSimulation()){
      m_leftDriveSim = m_leftLeader.getSimCollection();
      m_rightDriveSim = m_rightLeader.getSimCollection();

      m_pigeonSim = m_pigeon.getSimCollection();

      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);

      //Simulation class initialization
      m_odometry = new DifferentialDriveOdometry(m_pigeon.getRotation2d(), new Pose2d(7, 5, new Rotation2d(Units.degreesToRadians(120))));
      m_drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, 
        KitbotGearing.k10p71, 
        KitbotWheelSize.kSixInch, 
        null);
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }

  }

  public void stop(){
    m_leftLeader.stopMotor();
    m_rightLeader.stopMotor();
  }

  public void driveArcade(double fwd, double rot){
    m_leftLeader.set(ControlMode.PercentOutput, fwd + rot);
    m_rightLeader.set(ControlMode.PercentOutput, fwd - rot);
  }

  public void driveJoystick(double y, double x){
    double yout, xout;
    
    if(!isForwardSlow){
      yout = (y >= 0 ? 1 : -1) * (Constants.DriveConstants.kForwardSens*y*y + (1-Constants.DriveConstants.kForwardSens)*Math.abs(y));
    }else{
      if(y > 0){
        yout = 0.1;
      }else if(y < 0){
        yout = -0.1;
      }else{
        yout = 0;
      }
    }

    if(!isTurnSlow){
      xout = Constants.DriveConstants.kTurnMax* (x >= 0 ? 1 : -1) * (Constants.DriveConstants.kTurnSens*x*x + (1-Constants.DriveConstants.kTurnSens)*Math.abs(x));
    }else{
      if(x > 0){
        xout = 0.01;
      }else if(x < 0){
        xout = -0.01;
      }else{
        xout = 0;
      }
    }
    
    m_leftLeader.set(ControlMode.PercentOutput, yout + xout);
    m_rightLeader.set(ControlMode.PercentOutput, yout - xout);

  }

  public void activateSlowTurn(){
    isTurnSlow = true;
  }
  
  public void deactivateSlowTurn(){
    isTurnSlow = false;
  }

  public void activateSlowForward(){
    isForwardSlow = true;
  }

  public void deactivateSlowForward(){
    isForwardSlow = false;
  }


  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }


  public void resetEncoderSims(){
    m_leftEncoderSim.resetData();
    m_rightEncoderSim.resetData();
  }

  public void driveRaw(double left, double right){
    m_leftLeader.set(ControlMode.PercentOutput, left);
    m_rightLeader.set(ControlMode.PercentOutput, right);
  }

  public double getAvgEncoderDistance(){
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void refreshOdometry(){
    Pose2d pose = getPose();
    resetEncoders();
    m_drivetrainSim.setPose(pose);
    // m_pigeon.reset();
    // m_odometry.resetPosition(pose, new Rotation2d());
    m_odometry.resetPosition(pose, new Rotation2d(pose.getRotation().getRadians()));
    // m_odometry = new DifferentialDriveOdometry(m_pigeon.getRotation2d(), new Pose2d(7, 5, new Rotation2d(Units.degreesToRadians(120))));
    
    // m_pigeon.reset();
    // System.out.println(m_pigeon.getFusedHeading());
    // System.out.println(getHeading());
  }

  public void zeroHeading(){
    m_pigeon.reset();
  }

  public void resetOdometry(Pose2d pose){
    // m_pigeon.reset();
    // m_pigeon.setFusedHeading(120);
    resetEncoders();
    m_drivetrainSim.setPose(pose);
    m_odometry.resetPosition(pose, new Rotation2d());
  }

  public double getHeading() {
    return Math.IEEEremainder(m_pigeon.getAngle(), 360);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CompetitionView/Left Drive Motors", m_leftLeader.getMotorOutputPercent());
    SmartDashboard.putNumber("CompetitionView/Right Drive Motors", m_rightLeader.getMotorOutputPercent());
    SmartDashboard.putBoolean("CompetitionView/Slow Turn", isTurnSlow);
    SmartDashboard.putBoolean("CompetitionView/Slow Drive", isForwardSlow);

    SmartDashboard.putNumber("DriveSubsystem/Left Drive Motors", m_leftLeader.getMotorOutputPercent());
    SmartDashboard.putNumber("DriveSubsystem/Right Drive Motors", m_rightLeader.getMotorOutputPercent());
    SmartDashboard.putBoolean("DriveSubsystem/Slow Turn", isTurnSlow);
    SmartDashboard.putBoolean("DriveSubsystem/Slow Drive", isForwardSlow);

    

    if(RobotBase.isSimulation()){
      m_odometry.update(m_pigeon.getRotation2d(),
                        m_leftEncoder.getDistance(),
                        m_rightEncoder.getDistance());
      m_fieldSim.setRobotPose(m_odometry.getPoseMeters());


      SmartDashboard.putNumber("Heading", getHeading());
    }
  }

  @Override
  public void simulationPeriodic() {
    m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_drivetrainSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(), 
                              -m_rightDriveSim.getMotorOutputLeadVoltage());

    m_drivetrainSim.update(0.02);


    m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());

    m_pigeonSim.setRawHeading(m_drivetrainSim.getHeading().getDegrees());

    
  }
}
