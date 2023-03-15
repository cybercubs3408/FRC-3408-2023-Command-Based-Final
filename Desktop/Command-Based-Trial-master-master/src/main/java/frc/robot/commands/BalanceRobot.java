// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceRobot extends CommandBase {

  private final DriveSubsystem m_robotDriver;
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param drive The drive subsystem to use
   */
  public BalanceRobot(DriveSubsystem drive) {    

    m_robotDriver = drive;
    addRequirements(drive);

  }

  @Override 
  public void execute() {

    double value = m_robotDriver.getHeading2();
    System.out.println(value);
    m_robotDriver.stopMotors();

    if(Math.abs(value) + 1.2 > 2.5) {

      if (value > 2.5) {

        m_robotDriver.tankDrive(.2, .2);
        value = m_robotDriver.getHeading2();

      }

      else if (value < -2.5) {


        m_robotDriver.tankDrive(-.2, -.2);
        value = m_robotDriver.getHeading2();

      }
    }
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return true;
  }
}
