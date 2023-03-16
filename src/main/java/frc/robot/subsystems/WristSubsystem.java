package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  public static final double WRIST_PID_P = 0.5;

  public static final double WRIST_SPEED_DEG_PER_TICK = 1.0;

  public static final double MAX_PITCH_DPS = 45.0;
  public static final double MAX_ROLL_DPS = 45.0;
  public static final double SECONDS_PER_TICK = 0.02;

  private ShoulderSubsystem m_shoulder;

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
  private double m_positionA_deg = 0.0;
  private double m_positionB_deg = 0.0;
  private double m_desiredPitch_deg = 0.0;
  private double m_desiredRoll_deg = 0.0;

  private Logger logger = Logger.getInstance();

  private double m_initial_pitch_deg;

  public WristSubsystem(ShoulderSubsystem m_shoulder) {
    this.m_shoulder = m_shoulder;

    m_wristmotorA = new CANSparkMax(WristConstants.WristCanIDA, CANSparkMax.MotorType.kBrushless);
  
    if (m_wristmotorA != null) {
      m_pidControllerA = m_wristmotorA.getPIDController();

      m_wristRelativeEncoderA = m_wristmotorA.getEncoder();
      m_wristAbsoluteEncoderA = new DutyCycleEncoder(WristConstants.AbsoluteEncoderAChannel);

      m_pidControllerA.setP(WRIST_PID_P);

      m_pidControllerA.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
    }

    m_wristmotorB = new CANSparkMax(WristConstants.WristCanIDB, CANSparkMax.MotorType.kBrushless);

    if (m_wristmotorB != null) {
      m_pidControllerB = m_wristmotorB.getPIDController();

      m_wristRelativeEncoderB = m_wristmotorB.getEncoder();
      m_wristAbsoluteEncoderB = new DutyCycleEncoder(WristConstants.AbsoluteEncoderBChannel);

      m_pidControllerB.setP(WRIST_PID_P);

      m_pidControllerB.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
    }
  }

  public void changePitch(double deltaPitch_deg) {
    m_desiredPitch_deg += deltaPitch_deg;
  }

  public void changeRoll(double deltaRoll_deg) {
    m_desiredRoll_deg += deltaRoll_deg;
  }
  
  public double limitPitch(double pitch) {
    return Math.max(Math.min(pitch, WristConstants.kMaxPitch), WristConstants.kMinPitch);
  }

  public double limitRoll(double roll) {
    return Math.max(Math.min(roll, WristConstants.kMaxRoll), WristConstants.kMinRoll);
  }

  private double adjustAbsEncoder(double rawReading, double offset, boolean reversed) {
    double adjusted = rawReading - offset;

    while (adjusted > 0.5) {
      adjusted -= 1.0;
    }
    while (adjusted < -0.5) {
      adjusted += 1.0;
    }

    if (reversed) {
      adjusted = -adjusted;
    }

    return adjusted;
  }

  @Override
  public void periodic() {
    boolean updated_pr = false;
    periodicCycles++;
    if (periodicCycles == 150) {
      // abs encoders stable now (after 3 secs)
      // do homing
      double absPosA = adjustAbsEncoder(m_wristAbsoluteEncoderA.getAbsolutePosition(), WristConstants.absAOffset, false);
      double absPosB = adjustAbsEncoder(m_wristAbsoluteEncoderB.getAbsolutePosition(), WristConstants.absBOffset, true);

      m_pitch_deg = (360.0 * (absPosA + absPosB)) / 2.0;
      m_desiredPitch_deg = m_pitch_deg;
      m_roll_deg = (360.0 * (absPosA - absPosB)) / 2.0;
      m_desiredRoll_deg = m_roll_deg;

      m_positionA_deg = (m_pitch_deg + m_roll_deg);
      m_positionB_deg = (m_pitch_deg - m_roll_deg);
      
      m_wristRelativeEncoderA.setPosition(m_positionA_deg * -WristConstants.REVS_PER_OUTPUT_DEGREE_LEFT);
      m_wristRelativeEncoderB.setPosition(m_positionB_deg * WristConstants.REVS_PER_OUTPUT_DEGREE_RIGHT);

      m_initial_pitch_deg = m_pitch_deg;
    }


    else if (periodicCycles > 150){
      /*
      if (Math.abs(m_operator.getLeftY()) > WristConstants.JOYSTICK_DEADBAND) {
        pitch(m_operator.getLeftY() * -WRIST_SPEED_DEG_PER_TICK);
        updated_pr = true;
      }

      if (Math.abs(m_operator.getRightX()) > WristConstants.JOYSTICK_DEADBAND) {
        roll(m_operator.getRightX() * WRIST_SPEED_DEG_PER_TICK);
        updated_pr = true;
      }*/

      double delta_pitch_deg = m_desiredPitch_deg - m_pitch_deg;
      double delta_roll_deg = m_desiredRoll_deg - m_roll_deg;

      if (delta_pitch_deg > (MAX_PITCH_DPS * SECONDS_PER_TICK)) {
        delta_pitch_deg = MAX_PITCH_DPS * SECONDS_PER_TICK;
      }

      if (delta_pitch_deg < -(MAX_PITCH_DPS * SECONDS_PER_TICK)) {
        delta_pitch_deg = -(MAX_PITCH_DPS * SECONDS_PER_TICK);
      }

      if (delta_roll_deg > (MAX_ROLL_DPS * SECONDS_PER_TICK)) {
        delta_roll_deg = MAX_ROLL_DPS * SECONDS_PER_TICK;
      }

      if (delta_roll_deg < -(MAX_ROLL_DPS * SECONDS_PER_TICK)) {
        delta_roll_deg = -(MAX_ROLL_DPS * SECONDS_PER_TICK);
      }

      m_pitch_deg += delta_pitch_deg;
      m_roll_deg += delta_roll_deg;

      //TODO double adjustedPitch_deg = m_pitch_deg - m_shoulder.getCurrentDegrees();
      double adjustedPitch_deg = m_pitch_deg;

      adjustedPitch_deg = limitPitch(adjustedPitch_deg);
      m_roll_deg = limitRoll(m_roll_deg);

      m_positionA_deg = (adjustedPitch_deg + m_roll_deg);
      m_positionB_deg = (adjustedPitch_deg - m_roll_deg);

      m_pidControllerA.setReference(m_positionA_deg * -WristConstants.REVS_PER_OUTPUT_DEGREE_LEFT, ControlType.kPosition);
      m_pidControllerB.setReference(m_positionB_deg * WristConstants.REVS_PER_OUTPUT_DEGREE_RIGHT, ControlType.kPosition);
      
      if (m_wristmotorA.getOutputCurrent() > maxCurrentMotorA)
        maxCurrentMotorA = m_wristmotorA.getOutputCurrent();
      if (m_wristmotorB.getOutputCurrent() > maxCurrentMotorB)
        maxCurrentMotorB = m_wristmotorB.getOutputCurrent();
    }

    //logger.recordOutput("wrist.a.power", m_wristmotorA.get());
    //logger.recordOutput("wrist.b.power", m_wristmotorB.get());
    logger.recordOutput("wrist.pitch", m_pitch_deg);
    logger.recordOutput("wrist.roll", m_roll_deg);
    logger.recordOutput("wrist.a.position", m_positionA_deg);
    logger.recordOutput("wrist.b.position", m_positionB_deg);
    logger.recordOutput("wrist.a.reference", m_positionA_deg*-WristConstants.REVS_PER_OUTPUT_DEGREE_LEFT);
    logger.recordOutput("wrist.b.reference", m_positionB_deg*WristConstants.REVS_PER_OUTPUT_DEGREE_RIGHT);
    logger.recordOutput("wrist.a.relative", m_wristRelativeEncoderA.getPosition());
    logger.recordOutput("wrist.b.relative", m_wristRelativeEncoderB.getPosition());
    logger.recordOutput("wrist.a.current", m_wristmotorA.getOutputCurrent());
    logger.recordOutput("wrist.b.current", m_wristmotorB.getOutputCurrent());
    logger.recordOutput("wrist.a.absolute", m_wristAbsoluteEncoderA.getAbsolutePosition());
    logger.recordOutput("wrist.b.absolute", m_wristAbsoluteEncoderB.getAbsolutePosition());
  }

  public boolean isMoving() {
    return (m_pitch_deg != m_desiredPitch_deg) || (m_roll_deg != m_desiredRoll_deg);
  }
}
