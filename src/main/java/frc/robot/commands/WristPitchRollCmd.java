// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristPitchRollCmd extends CommandBase {
  /** Creates a new WristPitchRollCmd. */
  private WristSubsystem m_wrist;
  private double m_pitch;
  private double m_roll;
  public WristPitchRollCmd(WristSubsystem wristSubsystem, double pitch, double roll) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
    m_wrist = wristSubsystem;
    m_pitch = pitch;
    m_roll = roll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setPitchRoll(m_pitch, m_roll);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_wrist.isMoving();
  }
}
