package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase{
    
  WPI_TalonSRX intFlip1 = new WPI_TalonSRX(11);  
  WPI_TalonSRX intFlip2 = new WPI_TalonSRX(12);
  CANSparkMax intSpin1 = new CANSparkMax(13, MotorType.kBrushless);  
  CANSparkMax intSpin2 = new CANSparkMax(14, MotorType.kBrushless);  

  public IntakeSubsystem() {
  }
/**
 * Function for spinning the intake motors/intaking objects
 * @param power decimal power from -1.0 to 1.0 applied to the intake motor
 */
  public void runIntake(double power) {

    intSpin1.set(power);
    intSpin2.set(power);

  }
/**
 * Function to stop the spinning of the intake motors
 */
  public void stopIntake() {

    intSpin1.set(0);
    intSpin2.set(0);

  }
}
