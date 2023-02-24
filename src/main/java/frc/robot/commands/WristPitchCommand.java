// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristPitchCommand extends CommandBase {

  private final WristSubsystem m_wristSubsystem;
  private double m_delta_deg;

  public WristPitchCommand(WristSubsystem subsystem, double delta_deg) {
    addRequirements(subsystem);

    m_wristSubsystem = subsystem;
    m_delta_deg = delta_deg;    
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_wristSubsystem.pitch(m_delta_deg);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
