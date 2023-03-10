package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
    
    CANSparkMax t5 = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax t6 = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax t7 = new CANSparkMax(7, MotorType.kBrushless);

    RelativeEncoder m_flippyEncoder = t5.getEncoder();
    RelativeEncoder m_teleEncoder = t7.getEncoder();

    double[] weightDegrees = {0,2,3,4,5};
    double[] weightPowers = {0,2,3,4,5};

    double[] degreesPosition = {0,2,3,4,5};
    double[] encoderPosition = {0,2,3,4,5}; 

    double[] teleLength = {0,2,3,4,5};
    double[] teleEncoder = {0,2,3,4,5}; 

    public ArmSubsystem() {

    }
    /**
     * Rotates the arm  
     * @param joystick The controller used
     */
    public void translateRotLin(XboxController stick) {
        double rotPower = stick.getRawAxis(1);

        t5.set(-rotPower * (0.15));
        t6.set(-rotPower * (0.15));

        double linPower = stick.getRawAxis(5);

        t7.set(-linPower * (0.15));

    }

   public void rotateArm(double power) {

        t5.set(-power);
        t6.set(-power);

   }

   public void extendArm(double power) {

        t7.set(-power);

   }

   public void stopArm() {

        t5.set(0);
        t6.set(0);
        t7.set(0);

   }

   public void resetEncoders() {

        m_flippyEncoder.setPosition(0);
        m_teleEncoder.setPosition(0);

   }

   public double getFlippyEncoder() {

        return m_flippyEncoder.getPosition();

   }

   public double getTeleEncoder() {

        return m_teleEncoder.getPosition();

    }
}
