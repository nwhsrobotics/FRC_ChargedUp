// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;

public class WristTurnCommand extends CommandBase {

  private final WristSubsystem m_wristSubsystem;
  private double m_nextPosition;

  private boolean turningLeft;
  private boolean turningRight;

  public WristTurnCommand(WristSubsystem subsystem, double nextPosition) {
    addRequirements(subsystem);
    m_nextPosition = nextPosition;

    m_wristSubsystem = subsystem;
    
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(turningLeft == true) {
    m_wristSubsystem.turnLeft(m_nextPosition);
    }
    else if (turningRight == true) {
    m_wristSubsystem.turnRight(m_nextPosition);
    }

  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
