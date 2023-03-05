package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  private XboxController m_operator;

  private int periodicCycles = 0;

  //TODO private ShoulderSubsystem m_shoulder;

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

  public WristSubsystem(XboxController m_operator) { //TODO add shoulder here

    this.m_operator = m_operator;

    //TODO this.m_shoulder = m_shoulder;

    m_wristmotorA = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);
  
    if (m_wristmotorA != null) {
      m_pidControllerA = m_wristmotorA.getPIDController();

      m_wristRelativeEncoderA = m_wristmotorA.getEncoder();
      m_wristAbsoluteEncoderA = new DutyCycleEncoder(1);


      m_pidControllerA.setP(0.1);

      m_pidControllerA.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
    }

    m_wristmotorB = new CANSparkMax(13, CANSparkMax.MotorType.kBrushless);

    if (m_wristmotorB != null) {
      m_pidControllerB = m_wristmotorB.getPIDController();

      m_wristRelativeEncoderB = m_wristmotorB.getEncoder();
      m_wristAbsoluteEncoderB = new DutyCycleEncoder(2);

      m_pidControllerB.setP(0.1);

      m_pidControllerB.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
    }


  }
  
  public void pitch(double delta_deg) {
    if (m_pitch_deg + delta_deg <= WristConstants.kMaxPitch && m_pitch_deg + delta_deg >= WristConstants.kMinPitch) {
      m_pitch_deg += delta_deg;
    }
  }

  public void roll(double delta_deg) {
    if(m_roll_deg + delta_deg <= WristConstants.kMaxRoll && m_roll_deg + delta_deg >= WristConstants.kMinRoll) {
      m_roll_deg += delta_deg;
    }
  }

  @Override
  public void periodic() {
    periodicCycles++;
    if (periodicCycles == 150) {
      double absPosA = m_wristAbsoluteEncoderA.getAbsolutePosition();
      double incremDegA;
      if (absPosA > 1 - (0.5 - WristConstants.absAOffset)) {
        incremDegA = -360.0 * (1 - absPosA + WristConstants.absAOffset);
      }
      else {
        incremDegA = (absPosA - WristConstants.absAOffset) * 360.0;
      }

      double incrementalPosA = incremDegA * WristConstants.REVS_PER_OUTPUT_DEGREE;
      System.out.printf("========================== (%f - %f) * %f = %f\n\n", absPosA, WristConstants.absAOffset, WristConstants.WRIST_GEAR_RATIO, incrementalPosA);
      m_wristRelativeEncoderA.setPosition(incrementalPosA);
      SmartDashboard.putNumber("incremental Pos A", incrementalPosA);

      double absPosB = m_wristAbsoluteEncoderB.getAbsolutePosition();
      double incremDegB;
      if (absPosB > 1 - (0.5 - WristConstants.absBOffset)) {
        incremDegB = -360.0 * (1 - absPosB + WristConstants.absBOffset);
      } else {
        incremDegB = (absPosB - WristConstants.absBOffset) * 360.0;
      }

      double incrementalPosB = incremDegB * -WristConstants.REVS_PER_OUTPUT_DEGREE;
      m_wristRelativeEncoderB.setPosition(incrementalPosB);

      System.out.printf("======================= (%f - %f) * %f = %f\n\n", absPosB, WristConstants.absBOffset, WristConstants.WRIST_GEAR_RATIO, incrementalPosB);
      SmartDashboard.putNumber("incremental Pos B", incrementalPosB);

    }

    else if (periodicCycles > 150){
      //TODO m_pitch_deg = ShoulderConstants.kAngleRange - m_shoulder.m_desiredPos;
      if (m_operator.getLeftY() > 0.1) {
        pitch(m_operator.getLeftY()*5);
      } else if (m_operator.getLeftY() < -0.1) {
        pitch(m_operator.getLeftY()*5);
      }

      if (m_operator.getRightX() > 0.1){
        roll(m_operator.getRightX()*5);
      } else if (m_operator.getRightX() < -0.1){
        roll(m_operator.getRightX()*5);
      }
      
      m_positionA = (m_pitch_deg + m_roll_deg) * -WristConstants.REVS_PER_OUTPUT_DEGREE;
      m_positionB = (m_pitch_deg - m_roll_deg) * WristConstants.REVS_PER_OUTPUT_DEGREE;

      m_pidControllerA.setReference(m_positionA, ControlType.kPosition);
      m_pidControllerB.setReference(m_positionB, ControlType.kPosition);


      // Below here is all nonrelevant dashboard stuff  
      double absoultePositionA = m_wristAbsoluteEncoderA.getAbsolutePosition();
      double absolutePositionB = m_wristAbsoluteEncoderB.getAbsolutePosition();
      
      double relPositionA = m_wristRelativeEncoderA.getPosition();
      double relPositionB = m_wristRelativeEncoderB.getPosition();
      
      SmartDashboard.putNumber("posA", m_positionA);
      SmartDashboard.putNumber("posB", m_positionB);
      SmartDashboard.putNumber("rel a", relPositionA);
      SmartDashboard.putNumber("rel b", relPositionB);
      SmartDashboard.putNumber("abs a", absoultePositionA);
      SmartDashboard.putNumber("abs b", absolutePositionB);
      SmartDashboard.putNumber("pitch", m_pitch_deg);
      SmartDashboard.putNumber("roll", m_roll_deg);

      SmartDashboard.putNumber("Motor A current", m_wristmotorA.getOutputCurrent());
      SmartDashboard.putNumber("motor B current", m_wristmotorB.getOutputCurrent());
      if (m_wristmotorA.getOutputCurrent() > maxCurrentMotorA)
        maxCurrentMotorA = m_wristmotorA.getOutputCurrent();
      if(m_wristmotorB.getOutputCurrent() > maxCurrentMotorB)
        maxCurrentMotorB = m_wristmotorB.getOutputCurrent();

      SmartDashboard.putNumber("Max Motor A current", maxCurrentMotorA);
      SmartDashboard.putNumber("Motor B current", maxCurrentMotorB);
    }

    /*

    Logger logger = Logger.getInstance();
    logger.recordOutput("wrist.motors.a.power", m_wristmotorA.get());
    logger.recordOutput("wrist.motors.b.power", m_wristmotorB.get());
    logger.recordOutput("wrist.pitch", m_pitch_deg);
    logger.recordOutput("wrist.roll", m_roll_deg);
    logger.recordOutput("wrist.motors.a.position", m_positionA);
    logger.recordOutput("wrist.motors.b.position", m_positionB);
    logger.recordOutput("wrist.encoders.a.position", m_wristRelativeEncoderA.getPosition());
    logger.recordOutput("wrist.encoders.b.position", m_wristRelativeEncoderB.getPosition());*/
  }
}
