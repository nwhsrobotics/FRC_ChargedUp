// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.GrabberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

  private DoubleSolenoid m_grabber;
  private CANSparkMax PCM_CanID;
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    // Create a new DoubleSolenoid object for controlling a double solenoid on channels 0 and 1
    //Setting grabber variable to the actual solenoid controlling the pneumatics
    m_grabber = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    //Setting the current value of the solenoid to be off
    m_grabber.set(DoubleSolenoid.Value.kOff);
  }

  public void grabberTurnOff() {
  // Set the double solenoid to the off position
  m_grabber.set(DoubleSolenoid.Value.kOff);
  }
  public void grabberExtend(){
  // Extend the double solenoid grabber
  m_grabber.set(DoubleSolenoid.Value.kForward);
  }

  public void grabberRetract(){
  // Retract the double solenoid grabber
  m_grabber.set(DoubleSolenoid.Value.kReverse);  
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
