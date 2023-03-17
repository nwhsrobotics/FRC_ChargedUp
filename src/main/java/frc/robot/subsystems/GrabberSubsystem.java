package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GrabberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
  

  private DoubleSolenoid m_grabber;
  private Logger logger = Logger.getInstance();
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    m_grabber = new DoubleSolenoid(7, PneumaticsModuleType.CTREPCM, GrabberConstants.forwardChannel, GrabberConstants.reverseChannel);
  }

  public void grabberTurnOff() {
    m_grabber.set(DoubleSolenoid.Value.kOff);
    SmartDashboard.putString("Grabber", "Off");
    logger.recordOutput("grabber.state", "off");
  }

  public void grabberExtend() {
    m_grabber.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putString("Grabber", "Extended");
    logger.recordOutput("grabber.state", "extended");
  }

  public void grabberRetract() {
    m_grabber.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Grabber", "Retracted");
    logger.recordOutput("grabber.state", "retracted");
  }

  @Override
  public void periodic() {
  }
}
