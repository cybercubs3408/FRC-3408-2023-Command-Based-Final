package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TranslateLinear extends CommandBase{
    
private final ArmSubsystem m_robotArm;
private final Limelight m_limelight;

CANSparkMax t7 = new CANSparkMax(7, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

private XboxController rightStick;

public TranslateLinear(ArmSubsystem arm, Limelight limelight){
  
  m_limelight = limelight;
  m_robotArm = arm;
  addRequirements(arm);

}

public void execute(){
  
  double targetDistance = 0;
  double length = 0;
  double angle = 0;

  if (m_limelight.getPipeline() == 0 || m_limelight.getPipeline() == 2) {

    targetDistance = m_limelight.updateLimelightVariables(true, (m_limelight.targetHeight2 - m_limelight.limelightHeight));
    length = m_limelight.returnLength(m_limelight.postHeight2 - m_limelight.armHeight, targetDistance);
    angle = m_limelight.returnAngle(m_limelight.postHeight2 - m_limelight.armHeight, targetDistance);

  }

  else if (m_limelight.getPipeline() == 1 || m_limelight.getPipeline() == 3) {

    targetDistance = m_limelight.updateLimelightVariables(true, (m_limelight.targetHeight1 - m_limelight.limelightHeight));
    length = m_limelight.returnLength(m_limelight.postHeight1 - m_limelight.armHeight, targetDistance);
    angle = m_limelight.returnAngle(m_limelight.postHeight1 - m_limelight.armHeight, targetDistance);

  }

  double teleEncoder = m_limelight.teleEncoderRanges(length);
  double rotEncoder = m_limelight.flipEncoderRanges(270 - angle);
  double weightPower = m_limelight.weightPowersRanges(270 - angle);

  if (Math.abs(m_robotArm.getFlippyEncoder()) < rotEncoder) {

    m_robotArm.rotateArm(.3);

  }
  else if (Math.abs(m_robotArm.getFlippyEncoder()) >= rotEncoder) {

    m_robotArm.rotateArm(weightPower);

  }

  if (Math.abs(m_robotArm.getTeleEncoder()) < teleEncoder) {

    m_robotArm.extendArm(3);

  }

  else if (Math.abs(m_robotArm.getTeleEncoder()) >= rotEncoder) {

    m_robotArm.stopArm();

  }


  if (targetDistance - 7.0 > 48) {

    m_robotArm.stopTele();

  }
}

@Override
public boolean isFinished() {
  // End when the controller is at the reference.
  return true;
}
}
