package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendArmConstants;

public class ExtendArmSubsystem extends SubsystemBase {
  private XboxController xboxController;
  private CANSparkMax m_extendArmMotor1 = null;
  private SparkMaxPIDController m_pidController1 = null;
  private RelativeEncoder m_extendArmEncoder1 = null;
  private static double m_currentPos = 0.0;
  private static double m_desiredPos = 0.0;
  private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  private static final double TOTAL_DISTANCE = 25.0; // Revisit this value!!!
  private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  private static final double SPEED_ROT_PER_TICK = ((TOTAL_DISTANCE)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);
  private double m_gearRatio = 18.9;
  private double m_oneRotationLength = 1.504; // in inches Revisit this values!!!
  private boolean m_enabled = false;

  /** Creates a new ExtendArmSubsystem. */
  public ExtendArmSubsystem(XboxController m_controller) {
    xboxController = m_controller;
    m_extendArmMotor1 = new CANSparkMax(ExtendArmConstants.ExtendArmCanID24, CANSparkMax.MotorType.kBrushless);

    if (m_extendArmMotor1 != null) {
      m_pidController1 = m_extendArmMotor1.getPIDController();
      m_extendArmEncoder1 = m_extendArmMotor1.getEncoder();
      m_extendArmEncoder1.setPosition(0);

      m_pidController1.setP(ExtendArmConstants.kp);
      m_pidController1.setI(ExtendArmConstants.ki);
      m_pidController1.setD(ExtendArmConstants.kd);
      m_pidController1.setIZone(ExtendArmConstants.kIz);
      m_pidController1.setFF(ExtendArmConstants.kFFz);

      m_pidController1.setOutputRange(ExtendArmConstants.kMinOutput, ExtendArmConstants.kMaxOutput);
      m_pidController1.setReference(0.0, ControlType.kPosition);
      System.out.println("ExtendArmMotor1 initialized");
      m_enabled = true;
    }
  }

  public void setPos(double p_distInches) {
    m_desiredPos = ((p_distInches / m_oneRotationLength) * m_gearRatio);

    System.out.println("desiredPos: " + m_desiredPos);
  }

  @Override
  public void periodic() {
    if (xboxController.getPOV() == 90 && m_currentPos < 48) {
      setPos(m_currentPos + 2);
    }
    else if (xboxController.getPOV() == 270 && m_currentPos > -8) {
      setPos(m_currentPos - 2);
    }
    if (m_enabled == true) {
      if ((m_desiredPos > ((19 / m_oneRotationLength) * m_gearRatio))) {
        m_desiredPos = ((19 / m_oneRotationLength) * m_gearRatio);
      } else if (m_desiredPos < ((0 / m_oneRotationLength) * m_gearRatio)) {
        m_desiredPos = ((0 / m_oneRotationLength) * m_gearRatio);
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

      SmartDashboard.putNumber("Extending Arm", m_extendArmEncoder1.getPosition());
    } else {
      return;
    }
  }
}
