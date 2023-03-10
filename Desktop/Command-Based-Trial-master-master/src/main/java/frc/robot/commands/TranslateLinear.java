package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TranslateLinear extends CommandBase{
    
private final ArmSubsystem m_robotArmed;

CANSparkMax t7 = new CANSparkMax(7, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

private XboxController rightStick;

public TranslateLinear(ArmSubsystem arm){

    m_robotArmed = arm;
    addRequirements(arm);



}

public void execute(){
    double power = rightStick.getRawAxis(5);

    t7.set(power);

}

@Override
public boolean isFinished() {
  // End when the controller is at the reference.
  return true;
}
}
