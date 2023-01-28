// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {
  private final CANSparkMax shoulderMotor;
  
  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
    shoulderMotor = new CANSparkMax(20, CANSparkMax.MotorType.kBrushless);  
    SparkMaxPIDController pidController = shoulderMotor.getPIDController();
    pidController.setP(ShoulderConstants.kp);
    pidController.setI(ShoulderConstants.ki);
    pidController.setD(ShoulderConstants.kd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

