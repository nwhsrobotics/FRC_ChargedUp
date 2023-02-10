// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class WristSubsystem extends SubsystemBase {
  private CANSparkMax wristLeftMotor = null;
  private CANSparkMax wristRightMotor = null;

  private SparkMaxPIDController pidControllerLeft = null;
  private SparkMaxPIDController pidControllerRight = null;

  private RelativeEncoder wristLeftRelativeEncoder = null;
  private RelativeEncoder wristRightRelativeEncoder = null;
  
  public static double positionIncrease = 1.0;

  public WristSubsystem() {
    wristLeftMotor = new CANSparkMax(WristConstants.WristCanID60, CANSparkMax.MotorType.kBrushless);

    if (wristLeftMotor != null) {
      // getting PIDController instance from the wrist motor
      pidControllerLeft = wristLeftMotor.getPIDController();
      // getting the encoder instance from the wrist motor
      wristLeftRelativeEncoder = wristLeftMotor.getEncoder();
      // setting the encoder position to zero
      wristLeftRelativeEncoder.setPosition(0);

      // setting the P, I, and D values for the PIDController from the wristConstants
      pidControllerLeft.setP(WristConstants.kp);
      pidControllerLeft.setI(WristConstants.ki);
      pidControllerLeft.setD(WristConstants.kd);

      // setting the IZone and FF values for the PIDController from the WristConstants
      pidControllerLeft.setIZone(WristConstants.kIz);
      pidControllerLeft.setFF(WristConstants.kFFz);

      // setting the output range for the PIDController from the WristConstants
      pidControllerLeft.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidControllerLeft.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the Wrist motor 1
      System.out.println("WristMotor1 initialized");
    }

    // Initialize the second motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the Wrist motor with ID WristCanID21
    wristRightMotor = new CANSparkMax(WristConstants.WristCanID61, CANSparkMax.MotorType.kBrushless);

    // checking if the Wrist motor instance is not null
    if (wristRightMotor != null) {
      // getting PIDController instance from the Wrist motor
      pidControllerRight = wristRightMotor.getPIDController();
      // getting the encoder instance from the Wrist motor
      wristRightRelativeEncoder = wristRightMotor.getEncoder();
      // setting the encoder position to zero
      wristRightRelativeEncoder.setPosition(0);

      // setting the P, I, and D values for the PIDController from the WristConstants
      pidControllerRight.setP(WristConstants.kp);
      pidControllerRight.setI(WristConstants.ki);
      pidControllerRight.setD(WristConstants.kd);

      // setting the IZone and FF values for the PIDController from the WristConstants
      pidControllerRight.setIZone(WristConstants.kIz);
      pidControllerRight.setFF(WristConstants.kFFz);

      // setting the output range for the PIDController from the WristConstants
      pidControllerRight.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidControllerRight.setReference(0.0, ControlType.kPosition);
    }
  }
  
 
  public void turnLeft(double nextPosition) {
    pidControllerLeft.setReference(nextPosition, ControlType.kPosition);
    pidControllerRight.setReference(nextPosition, ControlType.kPosition);
    
  }

  public void turnRight(double nextPosition) {
    pidControllerLeft.setReference(-nextPosition, ControlType.kPosition);
    pidControllerRight.setReference(-nextPosition, ControlType.kPosition);
    
  }

  public void liftUp(double nextPosition) {
    pidControllerLeft.setReference(-nextPosition, ControlType.kPosition);
    pidControllerRight.setReference(-nextPosition, ControlType.kPosition);
    
  }

  public void lowerDown(double nextPosition) {
    pidControllerLeft.setReference(nextPosition, ControlType.kPosition);
    pidControllerRight.setReference(nextPosition, ControlType.kPosition);
    
  }

  @Override
  public void periodic() {
  }
}
