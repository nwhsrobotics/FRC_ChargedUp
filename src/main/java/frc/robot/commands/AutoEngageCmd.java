// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoEngageCmd extends CommandBase {
  /** Creates a new AutoEngageCmd. */

  private SwerveSubsystem m_swerve;
  private double CRITICAL_ANGLE = 10;
  private double ENGAGE_SPEED_MPS = 0.5;
  public AutoEngageCmd(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    m_swerve = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
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
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
