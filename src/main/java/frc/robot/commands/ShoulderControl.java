package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderControl extends CommandBase {
  private ShoulderSubsystem m_shoulder;
  private XboxController m_operator;
  
  public ShoulderControl(ShoulderSubsystem m_shoulder, XboxController m_operator) {
    this.m_shoulder = m_shoulder;
    this.m_operator = m_operator;
    addRequirements(m_shoulder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
