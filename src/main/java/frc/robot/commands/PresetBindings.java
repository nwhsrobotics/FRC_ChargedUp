// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PresetBindings extends CommandBase {
  private ExtendArmSubsystem m_extendArm;
  private ShoulderSubsystem m_shoulder;
  private XboxController m_operator;

  /** Creates a new PresetBindings. */
  public PresetBindings(ExtendArmSubsystem extendArm, ShoulderSubsystem shoulder, XboxController operator) {
    addRequirements(extendArm, shoulder);
    m_extendArm = extendArm;
    m_shoulder = shoulder;
    m_operator = operator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_operator.getRawButtonPressed(1)) {
      ParallelCommandGroup Inside = new ParallelCommandGroup(
          new InstantCommand(() -> m_shoulder.setPos_deg(0)),
          new InstantCommand(() -> m_extendArm.setPos_inch(0)));
    }
    if (m_operator.getRawButtonPressed(3)) {
      ParallelCommandGroup Ground = new ParallelCommandGroup(
          new InstantCommand(() -> m_shoulder.setPos_deg(25)),
          new InstantCommand(() -> m_extendArm.setPos_inch(5)));
    }
    if (m_operator.getRawButtonPressed(4)) {
      ParallelCommandGroup Middle = new ParallelCommandGroup(
          new InstantCommand(() -> m_shoulder.setPos_deg(40)),
          new InstantCommand(() -> m_extendArm.setPos_inch(10)));
    }
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
