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
    logger.recordOutput("Grabber", "Intialized");
  }

  public void grabberTurnOff() {
    m_grabber.set(DoubleSolenoid.Value.kOff);
    SmartDashboard.putString("Grabber", "Off");
    logger.recordOutput("Grabber", "Off");
  }

  public void grabberExtend() {
    m_grabber.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putString("Grabber", "Extended");
    logger.recordOutput("Grabber", "Extended");
  }

  public void grabberRetract() {
    m_grabber.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Grabber", "Retracted");
    logger.recordOutput("Grabber", "Retracted");
  }

  @Override
  public void periodic() {

  }
}
