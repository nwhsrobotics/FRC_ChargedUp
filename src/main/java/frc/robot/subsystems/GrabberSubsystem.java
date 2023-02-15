// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants.GrabberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

  private DoubleSolenoid m_grabber;
  private CANSparkMax PCM_CanID;
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {

    
    //Setting grabber variable to the actual solenoid controlling the pneumatics
    m_grabber = new DoubleSolenoid(0, null, 0, 0);
    //Setting the current value of the solenoid to be off
    m_grabber.set(DoubleSolenoid.Value.kOff);
  }

  public void grabberTurnOn(){


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
