// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveSubsystem extends SubsystemBase {

  CANSparkMax t1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax t2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax t3 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax t4 = new CANSparkMax(4, MotorType.kBrushless);

  MotorControllerGroup m_left = new MotorControllerGroup(t3, t4);
  MotorControllerGroup m_right = new MotorControllerGroup(t1,t2);

  DifferentialDrive m_drive = new DifferentialDrive(m_right, m_left);

  RelativeEncoder m_rightEncoder = t1.getEncoder();
  RelativeEncoder m_leftEncoder = t3.getEncoder();

  // The gyro sensor
  ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(getLeftEncoder(), getRightEncoder(), getLeftEncoder());

  PIDController leftPIDController = new PIDController(9.95,0,0);
  PIDController rightPIDController = new PIDController (9.95,0,0);

  Pose2d pose;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
    m_gyro.configCalTime(ADIS16470_IMU.CalibrationTime._128ms);
    m_gyro.reset();
    m_gyro.calibrate();

    t1.setInverted(true);
    t2.setInverted(true);
    t3.setInverted(true);

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param left left half movement 
   * @param right right half movement
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, -right);
  }

  



  public void tankDriveVolts(double leftVolts, double rightVolts) {

    m_left.setVoltage(leftVolts);
    m_right.setVoltage(rightVolts);
    m_drive.feed();

  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
    t2.getEncoder().getVelocity() / 7.9 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
    t4.getEncoder().getVelocity() / 7.9 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
    );
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public Pose2d getPose(){
      return pose;
  }

  public void setOutput (double leftVolts, double rightVolts){

    t2.set(leftVolts / 12);
    t4.set (rightVolts / 12);

  }

public PIDController getLeftPIDController(){
  return leftPIDController;
}

public PIDController getRightPIDController(){
  return rightPIDController;
}


  public void periodic(){
    pose = odometry.update(getHeading(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition() );
  }
  public void stopMotors() {

    m_left.set(0);
    m_right.set(0);

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return m_rightEncoder.getPosition();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading2() {
    return m_gyro.getYComplementaryAngle();
  }

  public void reset(){
    odometry.resetPosition(getHeading(), getLeftEncoder(), getHeading2(), pose);
  }
}
