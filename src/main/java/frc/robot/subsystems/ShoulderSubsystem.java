package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShoulderSubsystem extends SubsystemBase {
  private CANSparkMax m_shoulderMotor1 = null;
  public CANSparkMax m_shoulderMotor2 = null;
  private SparkMaxPIDController m_pidController1 = null;
  private SparkMaxPIDController m_pidController2 = null;
  private RelativeEncoder m_shoulderEncoder1 = null;
  private RelativeEncoder m_shoulderEncoder2 = null;
  private static double m_currentPos = 0.0;
  public static double m_desiredPos = 0.0;
  private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  private static final double TOTAL_DISTANCE = 61.1;
  private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  private static final double SPEED_ROT_PER_TICK = ((TOTAL_DISTANCE)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);

  private boolean m_enabled = false;

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
    m_shoulderMotor1 = new CANSparkMax(ShoulderConstants.ShoulderCanID20, CANSparkMax.MotorType.kBrushless);

    if (m_shoulderMotor1 != null) {
      m_pidController1 = m_shoulderMotor1.getPIDController();
      m_shoulderEncoder1 = m_shoulderMotor1.getEncoder();
      m_shoulderEncoder1.setPosition(0);

      m_pidController1.setP(ShoulderConstants.kp);
      m_pidController1.setI(ShoulderConstants.ki);
      m_pidController1.setD(ShoulderConstants.kd);
      m_pidController1.setIZone(ShoulderConstants.kIz);
      m_pidController1.setFF(ShoulderConstants.kFFz);

      m_pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
      m_pidController1.setReference(0.0, ControlType.kPosition);
      System.out.println("ShoulderMotor1 initialized");
    }

    m_shoulderMotor2 = new CANSparkMax(ShoulderConstants.ShoulderCanID21, CANSparkMax.MotorType.kBrushless);

    if (m_shoulderMotor2 != null) {
      m_pidController2 = m_shoulderMotor2.getPIDController();
      m_shoulderEncoder2 = m_shoulderMotor2.getEncoder();
      m_shoulderEncoder2.setPosition(0);

      m_pidController2.setP(ShoulderConstants.kp);
      m_pidController2.setI(ShoulderConstants.ki);
      m_pidController2.setD(ShoulderConstants.kd);
      m_pidController2.setIZone(ShoulderConstants.kIz);
      m_pidController2.setFF(ShoulderConstants.kFFz);

      m_pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
      m_pidController2.setReference(0.0, ControlType.kPosition);
      System.out.println("ShoulderMotor2 initialized");
      m_enabled = true;
    }
  }

  public void setPos(double p_degree) {
    m_desiredPos = ((p_degree / 360) * 200);
  }

  @Override
  public void periodic() {
    if (m_enabled == true) {
      if (m_desiredPos > ((110 / 360) * 200)) {
        m_desiredPos = ((110 / 360) * 200);
      } else if (m_desiredPos > ((0 / 360) * 200)) {
        m_desiredPos = ((0 / 360) * 200);
      }

      double distance = (m_desiredPos - m_currentPos);
      double delta = distance;

      if (delta > SPEED_ROT_PER_TICK) {
        delta = SPEED_ROT_PER_TICK;
      } else if(delta < -SPEED_ROT_PER_TICK) {
        delta = -SPEED_ROT_PER_TICK;
      }

      m_currentPos += delta;

      m_pidController1.setReference(m_currentPos, ControlType.kPosition);
      m_pidController2.setReference(-m_currentPos, ControlType.kPosition);

      SmartDashboard.putNumber("Shoulder 1 Position", m_shoulderEncoder1.getPosition());
      SmartDashboard.putNumber("Shoulder 2 Position", m_shoulderEncoder2.getPosition());
    } else {
      return;
    }
  }
}
