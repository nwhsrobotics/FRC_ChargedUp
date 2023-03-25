package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristJoystickCmd extends CommandBase {
  private WristSubsystem m_wrist;
  private XboxController m_operator;

  public WristJoystickCmd(WristSubsystem wristSubsystem, XboxController operatorController) {
    addRequirements(wristSubsystem);
    m_wrist = wristSubsystem;
    m_operator = operatorController;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (Math.abs(m_operator.getLeftY()) > WristConstants.JOYSTICK_DEADBAND) {
      m_wrist.changePitch(m_operator.getLeftY() * -WristSubsystem.WRIST_SPEED_DEG_PER_TICK);
    }

    if (Math.abs(m_operator.getRightX()) > WristConstants.JOYSTICK_DEADBAND) {
      m_wrist.changeRoll(m_operator.getRightX() * WristSubsystem.WRIST_SPEED_DEG_PER_TICK);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
