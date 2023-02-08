// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendArmConstants;

// ExtendArmSubsystem initializes and sets up two brushless motors and their associated encoders and PID controllers
public class ExtendArmSubsystem extends SubsystemBase {

  // Declare two instances of the CANSparkMax motor controller class
  private CANSparkMax extendArmMotor1 = null;
  private CANSparkMax extendArmMotor2 = null;
  // Declare two instances of the SparkMaxPIDController class
  private SparkMaxPIDController pidController1 = null;
  private SparkMaxPIDController pidController2 = null;
  // Declare two instances of the RelativeEncoder class
  private RelativeEncoder extendArmEncoder1 = null;
  private RelativeEncoder extendArmEncoder2 = null;
  // Set the default position for the shoulder when it is at the bottom
  private static final double DOWNPOS = 0.0; // Revisit this value!!!
  // Set the default position for the shoulder when it is at the top
  private static final double UPPOS = 90; // Revisit this value!!!
  // Set the number of ticks per second
  private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  // Set the time it takes for the shoulder to move from bottom to top
  private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  // Calculate the speed of rotation per tick
  private static final double SPEED_ROT_PER_TICK = ((UPPOS - DOWNPOS)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);
  // Set the initial shoulder position to be at the bottom or 0.0 position
  private double m_shoulderPos = 0.0;

  /** Creates a new ExtendArmSubsystem. */
  public ExtendArmSubsystem() {
    // Initialize the first motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID
    // ShoulderCanID20
    extendArmMotor1 = new CANSparkMax(
        ExtendArmConstants.ExtendArmCanID24,
        CANSparkMax.MotorType.kBrushless);
    // checking if the shoulder motor instance is not null
    if (extendArmMotor1 != null) {
      // getting PIDController instance from the shoulder motor
      pidController1 = extendArmMotor1.getPIDController();
      // getting the encoder instance from the shoulder motor
      extendArmEncoder1 = extendArmMotor1.getEncoder();
      // setting the encoder position to zero
      extendArmEncoder1.setPosition(0);

      // setting the P, I, and D values for the PIDController from the
      // ShoulderConstants
      pidController1.setP(ExtendArmConstants.kp);
      pidController1.setI(ExtendArmConstants.ki);
      pidController1.setD(ExtendArmConstants.kd);

      // setting the IZone and FF values for the PIDController from the
      // ShoulderConstants
      pidController1.setIZone(ExtendArmConstants.kIz);
      pidController1.setFF(ExtendArmConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      pidController1.setOutputRange(
          ExtendArmConstants.kMinOutput,
          ExtendArmConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidController1.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 1
      System.out.println("ShoulderMotor1 initialized");
    }

    // Initialize the second motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID
    // ShoulderCanID21
    extendArmMotor2 = new CANSparkMax(
        ExtendArmConstants.ExtendArmCanID25,
        CANSparkMax.MotorType.kBrushless);

    // checking if the shoulder motor instance is not null
    if (extendArmMotor2 != null) {
      // getting PIDController instance from the shoulder motor
      pidController2 = extendArmMotor2.getPIDController();
      // getting the encoder instance from the shoulder motor
      extendArmEncoder2 = extendArmMotor2.getEncoder();
      // setting the encoder position to zero
      extendArmEncoder2.setPosition(0);

      // setting the P, I, and D values for the PIDController from the
      // ShoulderConstants
      pidController2.setP(ExtendArmConstants.kp);
      pidController2.setI(ExtendArmConstants.ki);
      pidController2.setD(ExtendArmConstants.kd);

      // setting the IZone and FF values for the PIDController from the
      // ShoulderConstants
      pidController2.setIZone(ExtendArmConstants.kIz);
      pidController2.setFF(ExtendArmConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      pidController2.setOutputRange(
          ExtendArmConstants.kMinOutput,
          ExtendArmConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidController2.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 2
      System.out.println("ShoulderMotor2 initialized");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
