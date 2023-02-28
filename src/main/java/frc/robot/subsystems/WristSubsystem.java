// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//TODO: test the automatic pitch adjust

public class WristSubsystem extends SubsystemBase {
  private XboxController xboxController;

  //private ShoulderSubsystem m_shoulder;

  public CANSparkMax m_wristmotorA;
  public CANSparkMax m_wristmotorB;

  private SparkMaxPIDController m_pidControllerA = null;
  private SparkMaxPIDController m_pidControllerB = null;

  private RelativeEncoder m_wristRelativeEncoderA = null;
  private RelativeEncoder m_wristRelativeEncoderB = null;

  private DutyCycleEncoder m_wristAbsoluteEncoderA = new DutyCycleEncoder(1);
  private DutyCycleEncoder m_wristAbsoluteEncoderB = new DutyCycleEncoder(2);
  
  public double m_pitch_deg = 0.0;
  public double m_roll_deg = 0.0;
  private double m_positionA = 0.0;
  private double m_positionB = 0.0;

  public WristSubsystem(XboxController m_controller) {
    //TODO: Test everything

    xboxController = m_controller;

    //this.m_shoulder = m_shoulder;

    m_wristmotorA = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);
    m_wristmotorA.setIdleMode(IdleMode.kBrake);
  
    if (m_wristmotorA != null) {
      // getting PIDController instance from the wrist motor
      m_pidControllerA = m_wristmotorA.getPIDController();

      // getting the encoder instance from the wrist motor
      m_wristRelativeEncoderA = m_wristmotorA.getEncoder();
      //m_wristAbsoluteEncoderA = new DutyCycleEncoder(1);
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
    m_wristmotorB = new CANSparkMax(4, CANSparkMax.MotorType.kBrushless);
    m_wristmotorB.setIdleMode(IdleMode.kBrake);

    // checking if the Wrist motor instance is not null
    if (m_wristmotorB != null) {
      // getting PIDController instance from the Wrist motor
      m_pidControllerB = m_wristmotorB.getPIDController();
      // getting the encoder instance from the Wrist motor
      m_wristRelativeEncoderB = m_wristmotorB.getEncoder();
      //m_wristAbsoluteEncoderA = new DutyCycleEncoder(2);

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
  if(m_pitch_deg + delta_deg <= WristConstants.kMaxPitch && m_pitch_deg + delta_deg >= WristConstants.kMinPitch) {
    m_pitch_deg += delta_deg;
  }
}

public void roll(double delta_deg) {
  System.out.println("running");
  if(m_roll_deg + delta_deg <= WristConstants.kMaxRoll && m_roll_deg + delta_deg >= WristConstants.kMinRoll) {
    m_roll_deg += delta_deg;
  }
}

  @Override
  public void periodic() {
    //m_pitch_deg = ShoulderConstants.kAngleRange - m_shoulder.m_desiredPos;
    if (xboxController.getLeftY() > 0.15)
      pitch(0.1);
    else if (xboxController.getLeftY() < -0.15)
      pitch(-0.1);

    if (xboxController.getRightX() > 0.15)
      roll(2.5);
    else if (xboxController.getRightX() < -0.15)
      roll(-2.5);
      
      
    double absoultePositionA = m_wristAbsoluteEncoderA.getAbsolutePosition();
    double absolutePositionB = m_wristAbsoluteEncoderB.getAbsolutePosition();

    //System.out.println(absoultePositionA);
    //System.out.println(absolutePositionB);


    m_positionA = m_pitch_deg + m_roll_deg;
    m_positionB = (m_pitch_deg - m_roll_deg) * WristConstants.REVS_PER_OUTPUT_DEGREE;

    SmartDashboard.putNumber("motor A power", m_wristmotorA.get());
    SmartDashboard.putNumber("motor B power", m_wristmotorB.get());

    SmartDashboard.putNumber("m_pitch_deg", m_pitch_deg);
    SmartDashboard.putNumber("m_roll_deg", m_roll_deg);
    SmartDashboard.putNumber("m_positiona", m_positionA);
    SmartDashboard.putNumber("m_positionB", m_positionB);
    SmartDashboard.putNumber("Encoder A", m_wristRelativeEncoderA.getPosition());
    SmartDashboard.putNumber("Encoder B", m_wristRelativeEncoderB.getPosition());

    m_pidControllerA.setReference(m_positionA, ControlType.kPosition);
    m_pidControllerB.setReference(m_positionB, ControlType.kPosition);
    
  }
}
