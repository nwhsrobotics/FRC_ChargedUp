// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  private final PIDController pidControl;

  public WristSubsystem() {
    CANSparkMax wristLeftMoter = new CANSparkMax(WristConstants.CANID13, MotorType.kBrushless);
    CANSparkMax wristRightMoter = new CANSparkMax(WristConstants.CANID12, MotorType.kBrushless);

    pidControl = new PIDController(WristConstants.kp, WristConstants.ki, WristConstants.kd);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
