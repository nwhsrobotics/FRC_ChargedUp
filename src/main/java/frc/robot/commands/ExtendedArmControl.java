package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendArmSubsystem;

public class ExtendedArmControl extends CommandBase {
  private final ExtendArmSubsystem m_extendarmSubsystem;
  public double m_position;

  public ExtendedArmControl(ExtendArmSubsystem subsystem, double p_length) // length in inches
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_position = p_length;
    m_extendarmSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extendarmSubsystem.setPos(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_extendarmSubsystem.setPos(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}