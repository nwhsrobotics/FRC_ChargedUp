// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class shoulderCommand extends CommandBase {
  private final ShoulderSubsystem m_shoulderSubsystem;
  private boolean isOn = true;
  /** Creates a new shoulderCommand. */
  public shoulderCommand(ShoulderSubsystem subsystem) {
    addRequirements(subsystem);
    m_shoulderSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isOn) {
      m_shoulderSubsystem.resetPos();
    }
    else {
      m_shoulderSubsystem.maxPos();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    isOn = false;
    return false;

  }
}
