// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristJoystickCmd extends CommandBase {
  /** Creates a new WristJoystickCmd. */
  private WristSubsystem m_wrist;
  private XboxController m_operator;
  public WristJoystickCmd(WristSubsystem wristSubsystem, XboxController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
    m_wrist = wristSubsystem;
    m_operator = operatorController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_operator.getLeftY()) > WristConstants.JOYSTICK_DEADBAND) {
      m_wrist.changePitch(m_operator.getLeftY() * -WristSubsystem.WRIST_SPEED_DEG_PER_TICK);
    }

    if (Math.abs(m_operator.getRightX()) > WristConstants.JOYSTICK_DEADBAND) {
      m_wrist.changeRoll(m_operator.getRightX() * WristSubsystem.WRIST_SPEED_DEG_PER_TICK);
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
