// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveBase extends SubsystemBase {
 
  public WPI_TalonFX _left1 = new WPI_TalonFX(Constants.L1);
  public WPI_TalonFX _right1 = new WPI_TalonFX(Constants.R1);
  public WPI_TalonFX _left2 = new WPI_TalonFX(Constants.L2);
  public WPI_TalonFX _right2 = new WPI_TalonFX(Constants.R2);
  public WPI_TalonFX _left3 = new WPI_TalonFX(Constants.L3);
  public WPI_TalonFX _right3 = new WPI_TalonFX(Constants.R3);

  public double leftEncPos;
  public double rightEncPos; 
  public double leftEncVel;
  public double rightEncVel;

  public DifferentialDrive m_drive = new DifferentialDrive(_left1, _right1);

  public AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  
  public DifferentialDriveOdometry m_odometry;

  public DriveBase() {

    // _left1.configFactoryDefault();
		// _left2.configFactoryDefault();
		// _left3.configFactoryDefault();
		// _right1.configFactoryDefault();
		// _right2.configFactoryDefault();
		// _right3.configFactoryDefault();

    _left2.follow(_left1);
    _left3.follow(_left1);
    _right2.follow(_right1);
    _right3.follow(_right1);

    // _left1.configNeutralDeadband(0.001);
    // _right1.configNeutralDeadband(0.001);

    _left1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kDrivePIDIdx,Constants.kDriveTimeoutMs);
    _right1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kDrivePIDIdx,Constants.kDriveTimeoutMs);
    // _left2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kDrivePIDIdx,Constants.kDriveTimeoutMs);
    // _right2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kDrivePIDIdx,Constants.kDriveTimeoutMs);
    // _left3.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kDrivePIDIdx,Constants.kDriveTimeoutMs);
    // _right3.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,Constants.kDrivePIDIdx,Constants.kDriveTimeoutMs);
    
    // _left1.configNominalOutputForward(0,Constants.kDriveTimeoutMs);
    // _right1.configNominalOutputForward(0,Constants.kDriveTimeoutMs);
    // _left2.configNominalOutputForward(0,Constants.kDriveTimeoutMs);
    // _right2.configNominalOutputForward(0,Constants.kDriveTimeoutMs);
    // _left3.configNominalOutputForward(0,Constants.kDriveTimeoutMs);
    // _right3.configNominalOutputForward(0,Constants.kDriveTimeoutMs);

    // _left1.configNominalOutputReverse(0,Constants.kDriveTimeoutMs);
    // _right1.configNominalOutputReverse(0,Constants.kDriveTimeoutMs);
    // _left2.configNominalOutputReverse(0,Constants.kDriveTimeoutMs);
    // _right2.configNominalOutputReverse(0,Constants.kDriveTimeoutMs);
    // _left3.configNominalOutputReverse(0,Constants.kDriveTimeoutMs);
    // _right3.configNominalOutputReverse(0,Constants.kDriveTimeoutMs);

    // _left1.configPeakOutputForward(1,Constants.kDriveTimeoutMs);
    // _right1.configPeakOutputForward(1,Constants.kDriveTimeoutMs);
    // _left2.configPeakOutputForward(1,Constants.kDriveTimeoutMs);
    // _right2.configPeakOutputForward(1,Constants.kDriveTimeoutMs);
    // _left3.configPeakOutputForward(1,Constants.kDriveTimeoutMs);
    // _right3.configPeakOutputForward(1,Constants.kDriveTimeoutMs);

    // _left1.configPeakOutputReverse(-1,Constants.kDriveTimeoutMs);
    // _right1.configPeakOutputReverse(-1,Constants.kDriveTimeoutMs);
    // _left2.configPeakOutputReverse(-1,Constants.kDriveTimeoutMs);
    // _right2.configPeakOutputReverse(-1,Constants.kDriveTimeoutMs);
    // _left3.configPeakOutputReverse(-1,Constants.kDriveTimeoutMs);
    // _right3.configPeakOutputReverse(-1,Constants.kDriveTimeoutMs);

    // _left1.config_kF(Constants.kDrivePIDIdx,Constants.kGains_Vel.kF,Constants.kDriveTimeoutMs);
    // _right1.config_kF(Constants.kDrivePIDIdx,Constants.kGains_Vel.kF,Constants.kDriveTimeoutMs);
    // _left2.config_kF(Constants.kDrivePIDIdx,Constants.kGains_Vel.kF,Constants.kDriveTimeoutMs);
    // _right2.config_kF(Constants.kDrivePIDIdx,Constants.kGains_Vel.kF,Constants.kDriveTimeoutMs);
    // _left3.config_kF(Constants.kDrivePIDIdx,Constants.kGains_Vel.kF,Constants.kDriveTimeoutMs);
    // _right3.config_kF(Constants.kDrivePIDIdx,Constants.kGains_Vel.kF,Constants.kDriveTimeoutMs);

    // _left1.config_kP(Constants.kDrivePIDIdx,Constants.kGains_Vel.kP,Constants.kDriveTimeoutMs);
    // _right1.config_kP(Constants.kDrivePIDIdx,Constants.kGains_Vel.kP,Constants.kDriveTimeoutMs);
    // _left2.config_kP(Constants.kDrivePIDIdx,Constants.kGains_Vel.kP,Constants.kDriveTimeoutMs);
    // _right2.config_kP(Constants.kDrivePIDIdx,Constants.kGains_Vel.kP,Constants.kDriveTimeoutMs);
    // _left3.config_kP(Constants.kDrivePIDIdx,Constants.kGains_Vel.kP,Constants.kDriveTimeoutMs);
    // _right3.config_kP(Constants.kDrivePIDIdx,Constants.kGains_Vel.kP,Constants.kDriveTimeoutMs);

    // _left1.config_kI(Constants.kDrivePIDIdx,Constants.kGains_Vel.kI,Constants.kDriveTimeoutMs);
    // _right1.config_kI(Constants.kDrivePIDIdx,Constants.kGains_Vel.kI,Constants.kDriveTimeoutMs);
    // _left2.config_kI(Constants.kDrivePIDIdx,Constants.kGains_Vel.kI,Constants.kDriveTimeoutMs);
    // _right2.config_kI(Constants.kDrivePIDIdx,Constants.kGains_Vel.kI,Constants.kDriveTimeoutMs);
    // _left3.config_kI(Constants.kDrivePIDIdx,Constants.kGains_Vel.kI,Constants.kDriveTimeoutMs);
    // _right3.config_kI(Constants.kDrivePIDIdx,Constants.kGains_Vel.kI,Constants.kDriveTimeoutMs);

    // _left1.config_kD(Constants.kDrivePIDIdx,Constants.kGains_Vel.kD,Constants.kDriveTimeoutMs);
    // _right1.config_kD(Constants.kDrivePIDIdx,Constants.kGains_Vel.kD,Constants.kDriveTimeoutMs);
    // _left2.config_kD(Constants.kDrivePIDIdx,Constants.kGains_Vel.kD,Constants.kDriveTimeoutMs);
    // _right2.config_kD(Constants.kDrivePIDIdx,Constants.kGains_Vel.kD,Constants.kDriveTimeoutMs);
    // _left3.config_kD(Constants.kDrivePIDIdx,Constants.kGains_Vel.kD,Constants.kDriveTimeoutMs);
    // _right3.config_kD(Constants.kDrivePIDIdx,Constants.kGains_Vel.kD,Constants.kDriveTimeoutMs);
    
    //inverts the left motors
   _left1.setInverted(true);    
    _left2.setInverted(true);
    _left3.setInverted(true);       
    _right1.setInverted(false);
    _right2.setInverted(false);
    _right3.setInverted(false);
    
    //sets motor to brake motor
    _right1.setNeutralMode(NeutralMode.Brake);
    _right2.setNeutralMode(NeutralMode.Brake);
    _right3.setNeutralMode(NeutralMode.Brake);
    _left1.setNeutralMode(NeutralMode.Brake);
    _left2.setNeutralMode(NeutralMode.Brake);
    _left3.setNeutralMode(NeutralMode.Brake);
    
    // _right1.setNeutralMode(NeutralMode.Coast);
    // _right2.setNeutralMode(NeutralMode.Coast);
    // _right3.setNeutralMode(NeutralMode.Coast);
    // _left1.setNeutralMode(NeutralMode.Coast);
    // _left2.setNeutralMode(NeutralMode.Coast);
    // _left3.setNeutralMode(NeutralMode.Coast);

    _left1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.CurrentLimmit, 25, Constants.secondsForOpenRamp));
    _left2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.CurrentLimmit, 25, Constants.secondsForOpenRamp));
    _left3.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.CurrentLimmit, 25, Constants.secondsForOpenRamp));
    _right1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.CurrentLimmit, 25, Constants.secondsForOpenRamp));
    _right2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.CurrentLimmit, 25, Constants.secondsForOpenRamp));
    _right3.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.CurrentLimmit, 25, Constants.secondsForOpenRamp));
    
    m_odometry = new DifferentialDriveOdometry(getHeading());

    _left1.setStatusFramePeriod(1, 10);
    _right1.setStatusFramePeriod(1, 10);
    _left2.setStatusFramePeriod(1, 10);
    _right2.setStatusFramePeriod(1, 10);
    _left3.setStatusFramePeriod(1, 10);
    _right3.setStatusFramePeriod(1, 10);

    _left1.setStatusFramePeriod(3, 13100*100);
    _right1.setStatusFramePeriod(3, 13200*100);
    _left2.setStatusFramePeriod(3, 13300*100);
    _right2.setStatusFramePeriod(3, 13400*100);
    _left3.setStatusFramePeriod(3, 13500*100);
    _right3.setStatusFramePeriod(3, 13600*100);

    _left1.setStatusFramePeriod(4, 17100*100);
    _right1.setStatusFramePeriod(4, 17200*100);
    _left2.setStatusFramePeriod(4, 17300*100);
    _right2.setStatusFramePeriod(4, 17400*100);
    _left3.setStatusFramePeriod(4, 17500*100);
    _right3.setStatusFramePeriod(4, 17600*100);

    _left1.setStatusFramePeriod(8, 19100*100);
    _right1.setStatusFramePeriod(8, 19200*100);
    _left2.setStatusFramePeriod(8, 19300*100);
    _right2.setStatusFramePeriod(8, 19400*100);
    _left3.setStatusFramePeriod(8, 19500*100);
    _right3.setStatusFramePeriod(8, 19600*100);

    _left1.setStatusFramePeriod(14, 23100*100);
    _right1.setStatusFramePeriod(14, 23200*100);
    _left2.setStatusFramePeriod(14, 23300*100);
    _right2.setStatusFramePeriod(14, 23400*100);
    _left3.setStatusFramePeriod(14, 23500*100);
    _right3.setStatusFramePeriod(14, 23600*100);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    // may have to multiply velocity by 10 (change from 100ms to s?)
    leftEncPos = _left1.getSelectedSensorPosition() * Constants.kEncoderDistancePerPulse; 
    rightEncPos = _right1.getSelectedSensorPosition() * Constants.kEncoderDistancePerPulse;
    leftEncVel = _left1.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse;
    rightEncVel = _right1.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse;
    
    m_odometry.update(getHeading(), leftEncPos, rightEncPos);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {    
    return new DifferentialDriveWheelSpeeds(leftEncVel, rightEncVel);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(final Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getHeading());
  }

  public void resetEncoders() {
    leftEncPos = 0;
    rightEncPos = 0;
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }
  public double getOtherHeading(){
    return m_gyro.getPitch();
  }

  public double getAnotherHeading(){
    return m_gyro.getRoll();
    }
  // public double getHeadingActual() {
  //   return -m_gyro.getYaw() + Math.toDegrees(initTheta());
  // }`

  public void voltageControl(final double leftVolts, final double rightVolts) {
    _left1.setVoltage(leftVolts); 
    _right1.setVoltage(rightVolts); //used to be negative
    m_drive.feed(); 
  }
}
