// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;

public class WristRollCommand extends CommandBase {

  private final WristSubsystem m_wristSubsystem;
  //private double m_nextPosition;


  private double m_delta_deg;


  public WristRollCommand(WristSubsystem subsystem, double delta_deg) {
    addRequirements(subsystem);
    m_wristSubsystem = subsystem;
    m_delta_deg = delta_deg;
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_wristSubsystem.roll(m_delta_deg);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
