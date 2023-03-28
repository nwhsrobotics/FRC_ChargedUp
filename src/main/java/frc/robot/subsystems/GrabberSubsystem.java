package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GrabberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

  // Initialize DoubleSolenoid and Logger objects
  private DoubleSolenoid m_grabber;
  private Logger logger = Logger.getInstance();
  private Compressor m_compressor;

  public GrabberSubsystem() {
    // Initialize DoubleSolenoid object for pneumatics control
    m_grabber = new DoubleSolenoid(7, PneumaticsModuleType.CTREPCM, GrabberConstants.forwardChannel,
        GrabberConstants.reverseChannel);
    //for completely disabling the compressor for testing purposes so it doesnt make noises
    m_compressor = new Compressor(7, PneumaticsModuleType.CTREPCM); 
    m_compressor.disable();
  }

  public void grabberTurnOff() {
    // Set grabber state to Off
    m_grabber.set(DoubleSolenoid.Value.kOff);
    SmartDashboard.putString("Grabber", "Off"); // Update SmartDashboard with grabber state
    logger.recordOutput("grabber.state", "off"); // Record grabber state in logger
  }

  public void grabberExtend() {
    // Extend grabber
    m_grabber.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putString("Grabber", "Extended");
    logger.recordOutput("grabber.state", "extended");
  }

  public void grabberRetract() {
    // Retract grabber
    m_grabber.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Grabber", "Retracted");
    logger.recordOutput("grabber.state", "retracted");
  }

  @Override
  public void periodic() {
  }
}
