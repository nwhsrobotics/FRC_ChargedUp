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
  public CANSparkMax m_extendArmMotor1;
  private SparkMaxPIDController m_pidController1;
  private RelativeEncoder m_extendArmEncoder1;
  private double m_currentPos_inch = 0.0;
  private double m_desiredPos_inch = 0.0;
  //private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  //private static final double TOTAL_DISTANCE = 25.0; // Revisit this value!!! 100 would be best for 2.4 seconds and 50 would be 4.8 seconds 150 for 1.6 seconds
  //private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  private static final double GEAR_RATIO = 5.23*5.23;
  public static final double INCHES_PER_ROT = 2.0 * 24.0 * 5.0 / (25.4 * GEAR_RATIO); //stages * pulley teeth * mm per tooth / (mm per inch * gear ratio)
  private static final double SPEED_INCHES_PER_TICK = ExtendArmConstants.EXTEND_SPEED_IPS * ExtendArmConstants.SECONDS_PER_TICK;
  //private double m_oneRotationLength = 4.7; // in inches
  private boolean m_enabled = false;

  /** Creates a new ExtendArmSubsystem. */
  public ExtendArmSubsystem() {
    m_extendArmMotor1 = new CANSparkMax(33, CANSparkMax.MotorType.kBrushless);

    if (m_extendArmMotor1 != null) {
      //m_extendArmMotor1.setSmartCurrentLimit(1);
      m_pidController1 = m_extendArmMotor1.getPIDController();
      m_extendArmEncoder1 = m_extendArmMotor1.getEncoder();
      m_extendArmEncoder1.setPosition(1);

      m_pidController1.setP(ExtendArmConstants.kp);

      m_pidController1.setOutputRange(ExtendArmConstants.kMinOutput, ExtendArmConstants.kMaxOutput);
      //m_pidController1.setReference(0.0, ControlType.kPosition);
      m_enabled = true;
    }
  }

  /*/
  public void setPos(double p_distInches) {
    m_desiredPos = (((p_distInches / 2) / m_oneRotationLength) * m_gearRatio);      //basically takes the user input for inches and converts it to the first part of arm rotations because the caluclations are relative to the first part of the arm because the third part moves relatively to the first part and moves twice the distance of the first part of the arm with the same motor
  }*/

  @Override
  public void periodic() {
    if (m_enabled == true) {
      /*
      }*/
      if (m_desiredPos_inch > ExtendArmConstants.MAX_EXTEND_INCH) {      //if desired pos for arm is greater than 38 make it 38 and if less than 0 inches make it 0
        m_desiredPos_inch = ExtendArmConstants.MAX_EXTEND_INCH;
      } else if (m_desiredPos_inch < 0) {
        m_desiredPos_inch = 0;
      }

      double delta = (m_desiredPos_inch - m_currentPos_inch);
      
      if (delta > SPEED_INCHES_PER_TICK) {
        delta = SPEED_INCHES_PER_TICK;
      } else if (delta < -SPEED_INCHES_PER_TICK) {
        delta = -SPEED_INCHES_PER_TICK;
      }

      m_currentPos_inch += delta;
      double position_rot = m_currentPos_inch / INCHES_PER_ROT;
      m_pidController1.setReference(position_rot, ControlType.kPosition);

      SmartDashboard.putNumber("ExtendArm Rotations", position_rot);
      //SmartDashboard.putNumber("ExtendArm Inches", (m_extendArmEncoder1.getPosition() / m_gearRatio)* m_oneRotationLength * 2);
      SmartDashboard.putNumber("m_desiredPos_inch", m_desiredPos_inch);
      SmartDashboard.putNumber("m_currentPos_inch", m_currentPos_inch);
    } else {
      return;
    }
  }

  public void setPos_inch(double setPoint) {
    m_desiredPos_inch = setPoint;
  }

  public double getPos_inch() {
    return m_desiredPos_inch;
  }
}
