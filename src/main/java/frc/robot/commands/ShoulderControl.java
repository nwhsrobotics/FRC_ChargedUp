// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderControl extends CommandBase {
  private final ShoulderSubsystem m_shoulderSubsystem;
  private double m_position;
  /** Creates a new ShoulderControl. */
  public ShoulderControl(ShoulderSubsystem subsystem, double position) {
    addRequirements(subsystem);
    m_position = position;

    m_shoulderSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_shoulderSubsystem.setPos(m_position);
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
