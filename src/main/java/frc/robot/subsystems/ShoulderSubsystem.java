// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ShoulderSubsystem initializes and sets up two brushless motors and their associated encoders and PID controllers
public class ShoulderSubsystem extends SubsystemBase {
  // Declare two instances of the CANSparkMax motor controller class
  private CANSparkMax shoulderMotor1;
  private CANSparkMax shoulderMotor2;
  // Declare two instances of the SparkMaxPIDController class
  private SparkMaxPIDController pidController1;
  private SparkMaxPIDController pidController2;
  // Declare two instances of the RelativeEncoder class
  private RelativeEncoder shoulderEncoder1;
  private RelativeEncoder shoulderEncoder2;
  // Set the default position for the shoulder when it is at the bottom
  private static final double DOWNPOS = 0.0; // Revisit this value!!!
  // Set the default position for the shoulder when it is at the top
  private static final double UPPOS = 90.0; // Revisit this value!!!
  // Set the number of ticks per second
  private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  // Set the time it takes for the shoulder to move from bottom to top
  private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  // Calculate the speed of rotation per tick
  private static final double SPEED_ROT_PER_TICK = ((UPPOS - DOWNPOS)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);
  // Set the initial shoulder position to be at the bottom or 0.0 position
  private double m_shoulderPos = DOWNPOS;

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
    // Initialize the first motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID ShoulderCanID20
    //shoulderMotor1 = new CANSparkMax(ShoulderConstants.ShoulderCanID20, CANSparkMax.MotorType.kBrushless);
    // checking if the shoulder motor instance is not null
    if (shoulderMotor1 != null) {
      // getting PIDController instance from the shoulder motor
      pidController1 = shoulderMotor1.getPIDController();
      // getting the encoder instance from the shoulder motor
      shoulderEncoder1 = shoulderMotor1.getEncoder();
      // setting the encoder position to zero
      shoulderEncoder1.setPosition(0);

      // setting the P, I, and D values for the PIDController from the ShoulderConstants
      pidController1.setP(ShoulderConstants.kp);
      pidController1.setI(ShoulderConstants.ki);
      pidController1.setD(ShoulderConstants.kd);

      // setting the IZone and FF values for the PIDController from the ShoulderConstants
      pidController1.setIZone(ShoulderConstants.kIz);
      pidController1.setFF(ShoulderConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidController1.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 1
      System.out.println("ShoulderMotor1 initialized");
    }

    // Initialize the second motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID ShoulderCanID21
    shoulderMotor2 = new CANSparkMax(ShoulderConstants.ShoulderCanID21, CANSparkMax.MotorType.kBrushless);

    // checking if the shoulder motor instance is not null
    if (shoulderMotor2 != null) {
      // getting PIDController instance from the shoulder motor
      pidController2 = shoulderMotor2.getPIDController();
      // getting the encoder instance from the shoulder motor
      shoulderEncoder2 = shoulderMotor2.getEncoder();
      // setting the encoder position to zero
      shoulderEncoder2.setPosition(0);

      // setting the P, I, and D values for the PIDController from the ShoulderConstants
      pidController2.setP(ShoulderConstants.kp);
      pidController2.setI(ShoulderConstants.ki);
      pidController2.setD(ShoulderConstants.kd);

      // setting the IZone and FF values for the PIDController from the ShoulderConstants
      pidController2.setIZone(ShoulderConstants.kIz);
      pidController2.setFF(ShoulderConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidController2.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 2
      System.out.println("ShoulderMotor2 initialized");
    }
  }

  public void resetPos() {
    shoulderEncoder1.setPosition(DOWNPOS);
    shoulderEncoder2.setPosition(DOWNPOS);
    System.out.println("Position reset to default position");
  }

  public void maxPos() {
    shoulderEncoder1.setPosition(UPPOS);
    shoulderEncoder2.setPosition(UPPOS);
    System.out.println("Positon set to max position");
  }
  
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
