package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristJoystickCmd extends CommandBase {

  private WristSubsystem m_wrist;
  private XboxController m_operator;

  // Constructor that takes in the WristSubsystem and XboxController as parameters
  public WristJoystickCmd(WristSubsystem wristSubsystem, XboxController operatorController) {
    // Add the WristSubsystem as a requirement
    addRequirements(wristSubsystem);
    // Set the private variables to the input parameters
    m_wrist = wristSubsystem;
    m_operator = operatorController;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // If the left joystick Y value is greater than the deadband threshold
    if (Math.abs(m_operator.getLeftY()) > WristConstants.JOYSTICK_DEADBAND) {
      // Change the pitch of the wrist based on the joystick Y value
      m_wrist.changePitch(m_operator.getLeftY() * -WristSubsystem.WRIST_SPEED_DEG_PER_TICK);
    }

    // If the right joystick X value is greater than the deadband threshold
    if (Math.abs(m_operator.getRightX()) > WristConstants.JOYSTICK_DEADBAND) {
      // Change the roll of the wrist based on the joystick X value
      m_wrist.changeRoll(m_operator.getRightX() * WristSubsystem.WRIST_SPEED_DEG_PER_TICK);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // This command should never finish on its own    
    return false;
  }
}
