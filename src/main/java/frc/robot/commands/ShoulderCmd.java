package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCmd extends CommandBase {

  private ShoulderSubsystem m_shoulder;
  private double setPoint;

  public ShoulderCmd(ShoulderSubsystem m_shoulder, double setPoint) {
    this.m_shoulder = m_shoulder;
    this.setPoint = setPoint;
    addRequirements(m_shoulder);
  }

  @Override
  public void initialize() {
    m_shoulder.setPos_deg(setPoint); // Set the specific position of the ShoulderSubsystem to the setPoint value using the setPos_deg method
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return !m_shoulder.isMoving(); // Return true if the ShoulderSubsystem is not moving
  }
}
