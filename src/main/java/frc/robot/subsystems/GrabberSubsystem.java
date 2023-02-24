package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.GrabberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
  

  private DoubleSolenoid m_grabber;

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    m_grabber = new DoubleSolenoid(7, PneumaticsModuleType.CTREPCM, GrabberConstants.forwardChannel, GrabberConstants.reverseChannel);
    m_grabber.set(DoubleSolenoid.Value.kOff);
  }

  public void grabberTurnOff() {
    m_grabber.set(DoubleSolenoid.Value.kOff);
  }

  public void grabberExtend() {
    m_grabber.set(DoubleSolenoid.Value.kForward);
  }

  public void grabberRetract() {
    m_grabber.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
  }
}
