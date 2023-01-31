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

public class ShoulderSubsystem extends SubsystemBase {
  private CANSparkMax shoulderMotor1 = null;
  private CANSparkMax shoulderMotor2 = null;  
  private SparkMaxPIDController pidController1 = null;
  private SparkMaxPIDController pidController2 = null;
  private RelativeEncoder shoulderEncoder1 = null;
  private RelativeEncoder shoulderEncoder2 = null;
  private static final double DOWNPOS = 0.0; //Revisit values with cad!!!
  private static final double UPPOS = 90; //Revisit values with cad!!!
  private static final double TICKS_PER_SECOND = 50.0; //Revisit values!!!
  private static final double SECONDS_TO_MOVE = 1.0; //Revisit values!!!
  private static final double SPEED_ROT_PER_TICK = ((UPPOS - DOWNPOS))/ (SECONDS_TO_MOVE * TICKS_PER_SECOND);
  private double m_shoulderPos = 0.0;


 
  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
      shoulderMotor1 = new CANSparkMax(ShoulderConstants.ShoulderCanID20, CANSparkMax.MotorType.kBrushless);
      if(shoulderMotor1 != null){
        pidController1 = shoulderMotor1.getPIDController();
        shoulderEncoder1 = shoulderMotor1.getEncoder();
        shoulderEncoder1.setPosition(0);

        pidController1.setP(ShoulderConstants.kp);
        pidController1.setI(ShoulderConstants.ki);
        pidController1.setD(ShoulderConstants.kd);

        pidController1.setIZone(ShoulderConstants.kIz);
        pidController1.setFF(ShoulderConstants.kFFz);

        pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
        pidController1.setReference(0.0, ControlType.kPosition);
        System.out.println("ShoulderMotor1 initialized");
      }
      
      
      shoulderMotor2 = new CANSparkMax(ShoulderConstants.ShoulderCanID21, CANSparkMax.MotorType.kBrushless);
      
      if(shoulderMotor2 != null){
        pidController2 = shoulderMotor2.getPIDController(); 
        shoulderEncoder2 = shoulderMotor2.getEncoder();
        shoulderEncoder2.setPosition(0);
        
        pidController2.setP(ShoulderConstants.kp);
        pidController2.setI(ShoulderConstants.ki);
        pidController2.setD(ShoulderConstants.kd);

        pidController2.setIZone(ShoulderConstants.kIz);
        pidController2.setFF(ShoulderConstants.kFFz);

        pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
        pidController2.setReference(0.0, ControlType.kPosition);
        System.out.println("ShoulderMotor2 initialized");
      }

      }

  @Override
  public void periodic() {

      // This method will be called once per scheduler run
  }
}





