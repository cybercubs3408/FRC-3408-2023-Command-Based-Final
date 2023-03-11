package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem {

    CANSparkMax claw = new CANSparkMax(15, MotorType.kBrushless);
    RelativeEncoder clawEncoder = claw.getEncoder();

    public ClawSubsystem() {
    }

    /**
     * Sets encoder position to 0
     */
    public void resetEncoders() {

        clawEncoder.setPosition(0);

    }

    /**
     * Method to close claw/apply power in "close" directoin
     * @param power power value for the claw motor
     */
    public void closeClaw(double power) {

        claw.set(power);

    }

    /**
     * Method that stops the claw motor
     */
    public void stopClaw() {

        claw.set(0);

    }

    /** 
     * Method intended to release and open the claw back to a "ready to be used" state after picking up an object
     * TEST AND EDIT THE NEEDED ENCODER POSITION
     */
    public void openClaw() {

        stopClaw();
        resetEncoders();

        //Encoder value of open state, number likely must be changed
        while (Math.abs(clawEncoder.getPosition()) < .35) {
            claw.set(-.15);
        }
        stopClaw();
    }
}