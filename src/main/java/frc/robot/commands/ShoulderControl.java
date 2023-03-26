package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderControl extends CommandBase {
  private ShoulderSubsystem m_shoulderSubsystem;
  private XboxController m_controller;

  /**
   * Command for controlling the ShoulderSubsystem using an XboxController
   * 
   * @param subsystem  the ShoulderSubsystem to control
   * @param m_operator the XboxController to use for control
   */
  public ShoulderControl(ShoulderSubsystem subsystem, XboxController m_operator) {
    // Require the subsystem so it is not used elsewhere while this command is
    // running
    addRequirements(subsystem);
    // Store the subsystem and controller for later use
    m_shoulderSubsystem = subsystem;
    m_controller = m_operator;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Check if the up POV button is pressed and adjust the position if it is
    if (m_controller.getPOV() == 0) {
      m_shoulderSubsystem.changePos_deg(0.5);
      // Check if the down POV button is pressed and adjust the position if it is
    } else if (m_controller.getPOV() == 180) {
      m_shoulderSubsystem.changePos_deg(-0.5);
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
