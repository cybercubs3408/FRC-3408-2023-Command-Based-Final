// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Test comment
package frc.robot;

import static edu.wpi.first.wpilibj.PS4Controller.Button;

import java.util.Arrays;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BalanceRobot;
import frc.robot.commands.TranslateLinear;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final ArmSubsystem m_robotArm = new ArmSubsystem();
  private final ClawSubsystem m_robotClaw = new ClawSubsystem();

  // The driver's controller
  Joystick left = new Joystick(0);
  Joystick right = new Joystick(1);
  XboxController m_driverController = new XboxController(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick tank drive
    m_robotDrive.setDefaultCommand(
        // A split-stick tank drive command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.tankDrive(
                    //-m_driverController.getLeftY(), m_driverController.getRightY()),
                    left.getRawAxis(1), right.getRawAxis(1)),
            m_robotDrive));
    
    m_robotArm.setDefaultCommand(
      new RunCommand (
        () ->
          m_robotArm.translateRotLin(m_driverController),
          m_robotArm));
    
  }

  /**
     * Automatically binds the intakes movement to the reading of the xbox's ___ joystick
     * up on joystick moves intake up, down on joystick moves intake down
     */
    // m_robotIntake.setDefaultCommand(
    //     new RunCommand(
    //         () -> m_robotIntake.flipIntake(
    //                 m_driverController.getRawAxis(1)),
    //         m_robotIntake));

  

  public Command getAutonomousCommand() {

    var autoVoltageConstraints = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(0.00002, 0.00001, 0.000003), m_robotDrive.kinematics, 10);
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2),
    Units.feetToMeters(2));
    config.setKinematics(m_robotDrive.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(1.0,0, new Rotation2d())),
    config 
    );

  RamseteCommand command = new RamseteCommand(
    trajectory,
    m_robotDrive::getPose,
    new RamseteController(0, 0),
    m_robotDrive.getFeedforward(),
    m_robotDrive.getKinematics(),
    m_robotDrive::getSpeeds,
    m_robotDrive.getLeftPIDController(),
    m_robotDrive.getRightPIDController(),
    m_robotDrive::setOutput,
    m_robotDrive
  );

  return command;

  }

  public void reset(){
    m_robotDrive.reset();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // When the ___ button on the ____ is held run intake, when released turn off intake
    // new JoystickButton(controller, button)
    //     .onTrue(new InstantCommand(() -> m_robotIntake.runIntake(.3)))
    //     .onFalse(new InstantCommand(() -> m_robotIntake.stopIntake()));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
            .whileTrue(new BalanceRobot(m_robotDrive));

        //Press the X button on the Xbox controller to close the claw and hold an object
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
          .onTrue(new InstantCommand(() -> m_robotClaw.closeClaw(.3)))
          .onFalse(new InstantCommand(() -> m_robotClaw.closeClaw(.1)));
  
  }
}
