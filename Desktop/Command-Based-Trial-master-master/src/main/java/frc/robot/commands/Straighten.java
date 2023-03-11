package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.DriveSubsystem;

public class Straighten extends CommandBase{
    
    private final DriveSubsystem m_driveSubsystem;
    private final Limelight m_limelight;

    public Straighten(DriveSubsystem driveSubsystem, Limelight limelight) {

        m_driveSubsystem = driveSubsystem;
        m_limelight = limelight;
        addRequirements(driveSubsystem);

    }

    public void execute(){

        if (m_limelight.xOffEntry.getDouble(0) > 10) {

            m_driveSubsystem.tankDrive(0, 0.25);

        }

        else if (m_limelight.xOffEntry.getDouble(0) < 0) {

            m_driveSubsystem.tankDrive(0.25, 0);

        }

        else {

            m_driveSubsystem.stopMotors();

        }
    }
}
