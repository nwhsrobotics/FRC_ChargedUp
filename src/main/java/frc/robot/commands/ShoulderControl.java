package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderControl extends CommandBase {
  private final ShoulderSubsystem m_shoulderSubsystem;
  private double m_position;

  /** Creates a new ShoulderControl. */
  public ShoulderControl(ShoulderSubsystem subsystem, double p_degree) //degree angle
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_position = p_degree;
    m_shoulderSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_shoulderSubsystem.setPos(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_shoulderSubsystem.setPos(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
