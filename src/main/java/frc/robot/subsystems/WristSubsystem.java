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
//TODO: get arm position and adjust pitch

public class WristSubsystem extends SubsystemBase {
  private CANSparkMax m_wristmotorA = null;
  private CANSparkMax m_wristmotorB = null;

  private SparkMaxPIDController m_pidControllerA = null;
  private SparkMaxPIDController m_pidControllerB = null;

  private RelativeEncoder m_wristRelativeEncoderA = null;
  private RelativeEncoder m_wristRelativeEncoderB = null;
  
  private double m_pitch_deg = 0.0;
  private double m_roll_deg = 0.0;
  private double m_positionA = 0.0;
  private double m_positionB = 0.0;

  public WristSubsystem() {
    //TODO: Create absolute encoder for wrist motor
    //TODO: Create repositioning for those

    m_wristmotorA = new CANSparkMax(WristConstants.WristCanID60, CANSparkMax.MotorType.kBrushless);
  
    if (m_wristmotorA != null) {
      // getting PIDController instance from the wrist motor
      m_pidControllerA = m_wristmotorA.getPIDController();
      // getting the encoder instance from the wrist motor
      m_wristRelativeEncoderA = m_wristmotorA.getEncoder();
      // setting the encoder position to zero
      m_wristRelativeEncoderA.setPosition(0);

      // setting the P, I, and D values for the PIDController from the wristConstants
      m_pidControllerA.setP(WristConstants.kp);
      m_pidControllerA.setI(WristConstants.ki);
      m_pidControllerA.setD(WristConstants.kd);

      // setting the IZone and FF values for the PIDController from the WristConstants
      m_pidControllerA.setIZone(WristConstants.kIz);
      m_pidControllerA.setFF(WristConstants.kFFz);

      // setting the output range for the PIDController from the WristConstants
      m_pidControllerA.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      m_pidControllerA.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the Wrist motor 1
      System.out.println("WristMotor1 initialized");
    }

    // Initialize the second motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the Wrist motor with ID WristCanID21
    m_wristmotorB = new CANSparkMax(WristConstants.WristCanID61, CANSparkMax.MotorType.kBrushless);

    // checking if the Wrist motor instance is not null
    if (m_wristmotorB != null) {
      // getting PIDController instance from the Wrist motor
      m_pidControllerB = m_wristmotorB.getPIDController();
      // getting the encoder instance from the Wrist motor
      m_wristRelativeEncoderB = m_wristmotorB.getEncoder();
      // setting the encoder position to zero
      m_wristRelativeEncoderB.setPosition(0);

      // setting the P, I, and D values for the PIDController from the WristConstants
      m_pidControllerB.setP(WristConstants.kp);
      m_pidControllerB.setI(WristConstants.ki);
      m_pidControllerB.setD(WristConstants.kd);

      // setting the IZone and FF values for the PIDController from the WristConstants
      m_pidControllerB.setIZone(WristConstants.kIz);
      m_pidControllerB.setFF(WristConstants.kFFz);

      // setting the output range for the PIDController from the WristConstants
      m_pidControllerB.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      m_pidControllerB.setReference(0.0, ControlType.kPosition);
    }
  }
  
public void pitch(double delta_deg) {
  m_pitch_deg += delta_deg;
// TODO: limit pitch range
}

public void roll(double delta_deg) {
  m_roll_deg += delta_deg;
// TODO: limit roll range

}

  @Override
  public void periodic() {
    m_positionA = m_pitch_deg + m_roll_deg;
    m_positionB = (m_pitch_deg - m_roll_deg) * WristConstants.REVS_PER_OUTPUT_DEGREE;

  //TODO: convert deg to motor revolutions

    m_pidControllerA.setReference(m_positionA, ControlType.kPosition);
    m_pidControllerB.setReference(m_positionB, ControlType.kPosition);
    
  }
}
