package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoEngageCmd extends CommandBase {

  private SwerveSubsystem m_swerve;
  private double CRITICAL_ANGLE = 9;
  private double ENGAGE_SPEED_MPS = 0.5;

  public AutoEngageCmd(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    m_swerve = swerveSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double pitch = m_swerve.getPitchDeg();

    if (pitch > CRITICAL_ANGLE) {
      m_swerve.setSpeed(ENGAGE_SPEED_MPS);
    }

    else if (pitch < -CRITICAL_ANGLE) {
      m_swerve.setSpeed(-ENGAGE_SPEED_MPS);
    }

    else {
      m_swerve.setSpeed(0.0);
      // m_swerve.brake();
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
