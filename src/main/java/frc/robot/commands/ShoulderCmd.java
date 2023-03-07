package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCmd extends CommandBase {

  private final ShoulderSubsystem m_shoulder;
  private double setPoint;
  
  public ShoulderCmd(ShoulderSubsystem m_shoulder, double setPoint) {
    this.m_shoulder = m_shoulder;
    this.setPoint = setPoint;
    addRequirements(m_shoulder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shoulder.setPos_deg(setPoint);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
