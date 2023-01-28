// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {
  private final double kp = 1.0;
  private final double ki = 0.0;
  private final double kd = 0.0;
  private final PIDController pidControl;
  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
  pidControl = new PIDController(kp, ki, kd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
