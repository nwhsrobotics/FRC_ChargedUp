// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;

public class ShoulderSubsystem extends SubsystemBase {
  private final PIDController pidControl;
  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
  pidControl = new PIDController(ShoulderConstants.kp, ShoulderConstants.ki, ShoulderConstants.kd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
