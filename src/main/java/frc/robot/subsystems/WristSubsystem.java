package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  private XboxController m_operator;

  private int periodicCycles = 0;

  public CANSparkMax m_wristmotorA;
  public CANSparkMax m_wristmotorB;

  private SparkMaxPIDController m_pidControllerA = null;
  private SparkMaxPIDController m_pidControllerB = null;

  private double maxCurrentMotorA = 0;
  private double maxCurrentMotorB = 0;

  private RelativeEncoder m_wristRelativeEncoderA = null;
  private RelativeEncoder m_wristRelativeEncoderB = null;

  private DutyCycleEncoder m_wristAbsoluteEncoderA;
  private DutyCycleEncoder m_wristAbsoluteEncoderB;
  
  public double m_pitch_deg = 0.0;
  public double m_roll_deg = 0.0;
  private double m_positionA = 0.0;
  private double m_positionB = 0.0;

  private Logger logger = Logger.getInstance();

  public WristSubsystem(XboxController m_operator) {

    this.m_operator = m_operator;

    m_wristmotorA = new CANSparkMax(WristConstants.WristCanIDA, CANSparkMax.MotorType.kBrushless);
  
    if (m_wristmotorA != null) {
      m_pidControllerA = m_wristmotorA.getPIDController();

      m_wristRelativeEncoderA = m_wristmotorA.getEncoder();
      m_wristAbsoluteEncoderA = new DutyCycleEncoder(WristConstants.AbsoluteEncoderAChannel);

      m_pidControllerA.setP(0.1);

      m_pidControllerA.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
    }

    m_wristmotorB = new CANSparkMax(WristConstants.WristCanIDB, CANSparkMax.MotorType.kBrushless);

    if (m_wristmotorB != null) {
      m_pidControllerB = m_wristmotorB.getPIDController();

      m_wristRelativeEncoderB = m_wristmotorB.getEncoder();
      m_wristAbsoluteEncoderB = new DutyCycleEncoder(WristConstants.AbsoluteEncoderBChannel);

      m_pidControllerB.setP(0.1);

      m_pidControllerB.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
    }
  }
  
  public void pitch(double delta_deg) {
    m_pitch_deg = Math.max(Math.min(m_pitch_deg + delta_deg, WristConstants.kMaxPitch), WristConstants.kMinPitch);
  }

  public void roll(double delta_deg) {
    m_roll_deg = Math.max(Math.min(m_roll_deg + delta_deg, WristConstants.kMaxRoll), WristConstants.kMinRoll);
  }

  private double adjustAbsEncoder(double rawReading, double offset) {
    double adjusted = rawReading - offset;

    if (adjusted > 1.0) {
      adjusted -= 1.0;
    }
    else if (adjusted < 0) {
      adjusted += 1.0;
    }

    return adjusted;
  }

  @Override
  public void periodic() {
    periodicCycles++;
    if (periodicCycles == 150) {
      // abs encoders stable now (after 3 secs)
      // do homing
      double absPosA = adjustAbsEncoder(m_wristAbsoluteEncoderA.getAbsolutePosition(), WristConstants.absAOffset);
      double absPosB = adjustAbsEncoder(m_wristAbsoluteEncoderB.getAbsolutePosition(), WristConstants.absBOffset);

      m_roll_deg = (360.0 * (absPosA + absPosB)) / 2.0;
      m_pitch_deg = (360.0 * (absPosA - absPosB)) / 2.0;

      if (m_roll_deg > 180.0) { m_roll_deg -= 360.0; }
      if (m_roll_deg < -180.0) { m_roll_deg += 360.0; }
      if (m_pitch_deg > 180.0) { m_pitch_deg -= 360.0; }
      if (m_pitch_deg < -180.0) { m_pitch_deg += 360.0; }

      m_positionA = (m_pitch_deg + m_roll_deg) * -WristConstants.REVS_PER_OUTPUT_DEGREE;
      m_positionB = (m_pitch_deg - m_roll_deg) * WristConstants.REVS_PER_OUTPUT_DEGREE;
      
      m_wristRelativeEncoderA.setPosition(m_positionA);
      m_wristRelativeEncoderB.setPosition(m_positionB);
    }

    else if (periodicCycles > 150){
      //TODO m_pitch_deg = ShoulderConstants.kAngleRange - m_shoulder.m_desiredPos;
      if (Math.abs(m_operator.getLeftY()) > WristConstants.JOYSTICK_DEADBAND) {
        pitch(m_operator.getLeftY() * 5);
      }

      if (Math.abs(m_operator.getRightX()) > WristConstants.JOYSTICK_DEADBAND) {
        roll(m_operator.getRightX() * 5);
      }
      
      m_positionA = (m_pitch_deg + m_roll_deg) * -WristConstants.REVS_PER_OUTPUT_DEGREE;
      m_positionB = (m_pitch_deg - m_roll_deg) * WristConstants.REVS_PER_OUTPUT_DEGREE;

      m_pidControllerA.setReference(m_positionA, ControlType.kPosition);
      m_pidControllerB.setReference(m_positionB, ControlType.kPosition);
      
      if (m_wristmotorA.getOutputCurrent() > maxCurrentMotorA)
        maxCurrentMotorA = m_wristmotorA.getOutputCurrent();
      if (m_wristmotorB.getOutputCurrent() > maxCurrentMotorB)
        maxCurrentMotorB = m_wristmotorB.getOutputCurrent();
    }

    logger.recordOutput("wrist.a.power", m_wristmotorA.get());
    logger.recordOutput("wrist.b.power", m_wristmotorB.get());
    logger.recordOutput("wrist.pitch", m_pitch_deg);
    logger.recordOutput("wrist.roll", m_roll_deg);
    logger.recordOutput("wrist.a.position", m_positionA);
    logger.recordOutput("wrist.b.position", m_positionB);
    logger.recordOutput("wrist.a.relative", m_wristRelativeEncoderA.getPosition());
    logger.recordOutput("wrist.b.relative", m_wristRelativeEncoderB.getPosition());
    logger.recordOutput("wrist.a.current", m_wristmotorA.getOutputCurrent());
    logger.recordOutput("wrist.b.current", m_wristmotorB.getOutputCurrent());
    logger.recordOutput("wrist.a.absolute", m_wristAbsoluteEncoderA.getAbsolutePosition());
    logger.recordOutput("wrist.b.absolute", m_wristAbsoluteEncoderB.getAbsolutePosition());
  }
}
