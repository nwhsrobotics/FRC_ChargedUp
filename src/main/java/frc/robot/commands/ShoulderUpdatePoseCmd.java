// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderUpdatePoseCmd extends CommandBase {
  private ShoulderSubsystem m_shoulder;
  private int m_target;
  /** Creates a new ShoulderUpdatePoseCmd. */
  public ShoulderUpdatePoseCmd(ShoulderSubsystem shoulderSubsystem, int targetPose) {
    addRequirements(shoulderSubsystem);
    m_shoulder = shoulderSubsystem;
    m_target = targetPose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shoulder.setCurrentPose(m_target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
