package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderControl extends CommandBase {
  private ShoulderSubsystem m_shoulderSubsystem;
  private XboxController m_controller;
  
  public ShoulderControl(ShoulderSubsystem subsystem, XboxController m_operator) {
    addRequirements(subsystem);
    m_shoulderSubsystem = subsystem;
    m_controller = m_operator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_controller.getPOV() == 0) {
      //System.out.println("up");
      m_shoulderSubsystem.changePos_deg(0.5);  
    } else if (m_controller.getPOV() == 180) {
      //System.out.println("down");
      m_shoulderSubsystem.changePos_deg(-0.5); 
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
