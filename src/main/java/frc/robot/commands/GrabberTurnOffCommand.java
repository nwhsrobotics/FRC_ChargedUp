package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberTurnOffCommand extends CommandBase {

  private final GrabberSubsystem m_grabberSubsystem;

  /** Creates a new GrabberTurnOffCommand. */
  public GrabberTurnOffCommand(GrabberSubsystem subsystem) {
    addRequirements(subsystem);
    m_grabberSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_grabberSubsystem.grabberTurnOff();
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
